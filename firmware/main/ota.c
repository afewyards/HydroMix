#include "ota.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_timer.h"

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
