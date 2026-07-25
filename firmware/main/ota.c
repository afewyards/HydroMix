#include "ota.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_zigbee_core.h"

/* Bounded fallback: a flaky/faulted sensor must never cause a permanent rollback loop.
 * If a good sweep hasn't happened this long after rejoin, validate anyway. */
#define VALIDATE_FALLBACK_MS (10u * 60u * 1000u)

static const char *TAG = "ota";
static bool s_pending = false;    /* image awaiting validation */
static bool s_joined = false;
static bool s_swept  = false;
static esp_timer_handle_t s_fallback_timer;

static void mark_valid_now(const char *reason){
    if (!s_pending) return;
    esp_ota_mark_app_valid_cancel_rollback();
    s_pending = false;
    ESP_LOGI(TAG, "OTA image validated, rollback cancelled (%s)", reason);
}

static void maybe_validate(void){
    if (s_pending && s_joined && s_swept) mark_valid_now("rejoin + good sweep");
}

static void fallback_timer_cb(void *arg){
    if (s_pending) mark_valid_now("bounded fallback after rejoin");
}

void ota_init(void){
    esp_ota_img_states_t st;
    const esp_partition_t *run = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        s_pending = true;
        ESP_LOGW(TAG, "image pending verify: awaiting rejoin + good sweep");
    }
    const esp_timer_create_args_t targs = { .callback = fallback_timer_cb, .name = "ota_fallback" };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_fallback_timer));
}

void ota_note_joined(void){
    s_joined = true;
    if (s_pending) {
        esp_timer_stop(s_fallback_timer);   /* no-op if not currently running; ignore result */
        ESP_ERROR_CHECK(esp_timer_start_once(s_fallback_timer, (uint64_t)VALIDATE_FALLBACK_MS * 1000ULL));
    }
    maybe_validate();
}
void ota_note_good_sweep(void){ s_swept = true; maybe_validate(); }

/* ---------------- Zigbee OTA client (image transfer) ----------------
 *
 * The OTA cluster is declared in zigbee.c; this is the half that actually
 * receives an image and writes it into the inactive OTA slot. The rollback
 * gating above still applies afterwards: the newly booted image only cancels
 * its rollback once it rejoins and completes a clean sweep, or the bounded
 * fallback fires.
 *
 * The stack strips the 56-byte ZCL OTA file header before handing us payloads,
 * but NOT the 6-byte sub-element header (2-byte tag id + 4-byte length) that
 * precedes the image bytes in the first block. Writing those 6 bytes would
 * corrupt the image, so the first payload is trimmed. */
#define OTA_SUBELEMENT_HDR_LEN 6

static esp_ota_handle_t       s_dl_handle;
static const esp_partition_t *s_dl_part;
static bool                   s_dl_active;
static bool                   s_dl_trimmed;   /* sub-element header consumed */
static uint32_t               s_dl_written;
static uint32_t               s_dl_total;

static void download_abort(const char *why)
{
    if (s_dl_active) {
        esp_ota_abort(s_dl_handle);
        s_dl_active = false;
    }
    ESP_LOGW(TAG, "OTA download aborted: %s", why);
}

esp_err_t ota_zcl_handle(const void *msg)
{
    const esp_zb_zcl_ota_upgrade_value_message_t *m = msg;
    if (!m) return ESP_ERR_INVALID_ARG;

    switch (m->upgrade_status) {
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START: {
        if (s_dl_active) download_abort("restarted while a download was in flight");
        s_dl_part = esp_ota_get_next_update_partition(NULL);
        if (!s_dl_part) {
            ESP_LOGE(TAG, "no OTA partition available");
            return ESP_FAIL;
        }
        esp_err_t err = esp_ota_begin(s_dl_part, OTA_WITH_SEQUENTIAL_WRITES, &s_dl_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
            return err;
        }
        s_dl_active = true; s_dl_trimmed = false; s_dl_written = 0; s_dl_total = 0;
        ESP_LOGI(TAG, "OTA download started into '%s'", s_dl_part->label);
        return ESP_OK;
    }

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE: {
        if (!s_dl_active) return ESP_FAIL;
        const uint8_t *data = m->payload;
        uint16_t len = m->payload_size;
        s_dl_total = m->ota_header.image_size;

        if (!s_dl_trimmed) {
            if (len <= OTA_SUBELEMENT_HDR_LEN) return ESP_OK;   /* wait for more */
            data += OTA_SUBELEMENT_HDR_LEN;
            len  -= OTA_SUBELEMENT_HDR_LEN;
            s_dl_trimmed = true;
        }

        esp_err_t err = esp_ota_write(s_dl_handle, data, len);
        if (err != ESP_OK) {
            download_abort(esp_err_to_name(err));
            return err;
        }
        s_dl_written += len;
        return ESP_OK;
    }

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
        ESP_LOGI(TAG, "OTA image received: %lu bytes written (header size %lu)",
                 (unsigned long)s_dl_written, (unsigned long)s_dl_total);
        return ESP_OK;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
        return ESP_OK;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH: {
        if (!s_dl_active) return ESP_FAIL;
        esp_err_t err = esp_ota_end(s_dl_handle);
        s_dl_active = false;
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
            return err;
        }
        err = esp_ota_set_boot_partition(s_dl_part);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "set_boot_partition failed: %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGW(TAG, "OTA image applied, rebooting into '%s'", s_dl_part->label);
        esp_restart();
        return ESP_OK;   /* not reached */
    }

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
        download_abort("server or stack aborted");
        return ESP_OK;

    default:
        return ESP_OK;
    }
}
