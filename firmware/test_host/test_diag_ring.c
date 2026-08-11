#include "unity.h"
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "ctrl_core/diag_ring.h"

void setUp(void){} void tearDown(void){}

typedef struct { uint32_t a, b; } payload_t;
typedef struct {
    diag_hdr_t hdr;
    payload_t  cur;
    payload_t  hist[DIAG_RING_DEPTH];
} store_t;

#define MAGIC 0xABCD1234u

void test_cold_boot_zeroes_and_seeds(void){
    store_t s;
    memset(&s, 0xAA, sizeof s);          /* garbage, as uninitialised RTC RAM would be */
    bool warm = diag_ring_warm(&s, sizeof s, MAGIC);
    TEST_ASSERT_FALSE(warm);
    TEST_ASSERT_EQUAL_UINT32(MAGIC, s.hdr.magic);
    TEST_ASSERT_EQUAL_UINT32(1, s.hdr.seq);
    TEST_ASSERT_EQUAL_UINT32(0, s.cur.a);            /* payload wiped */
    TEST_ASSERT_EQUAL_UINT32(0, s.hist[DIAG_RING_DEPTH-1].b);
}

void test_warm_boot_bumps_seq_and_keeps_payload(void){
    store_t s;
    memset(&s, 0, sizeof s);
    diag_ring_warm(&s, sizeof s, MAGIC);             /* cold: seq = 1 */
    s.cur.a = 42;
    bool warm = diag_ring_warm(&s, sizeof s, MAGIC); /* warm: seq = 2 */
    TEST_ASSERT_TRUE(warm);
    TEST_ASSERT_EQUAL_UINT32(2, s.hdr.seq);
    TEST_ASSERT_EQUAL_UINT32(42, s.cur.a);           /* NOT wiped */
    diag_ring_warm(&s, sizeof s, MAGIC);
    TEST_ASSERT_EQUAL_UINT32(3, s.hdr.seq);
}

void test_wrong_magic_is_treated_as_cold(void){
    store_t s;
    memset(&s, 0, sizeof s);
    s.hdr.magic = 0xDEADBEEFu;
    s.cur.a = 99;
    TEST_ASSERT_FALSE(diag_ring_warm(&s, sizeof s, MAGIC));
    TEST_ASSERT_EQUAL_UINT32(0, s.cur.a);
}

void test_shift_moves_history_down_and_leaves_slot_zero(void){
    payload_t h[4] = { {1,1}, {2,2}, {3,3}, {4,4} };
    diag_ring_shift(h, sizeof h[0], 4);
    TEST_ASSERT_EQUAL_UINT32(1, h[0].a);   /* untouched, caller overwrites */
    TEST_ASSERT_EQUAL_UINT32(1, h[1].a);
    TEST_ASSERT_EQUAL_UINT32(2, h[2].a);
    TEST_ASSERT_EQUAL_UINT32(3, h[3].a);   /* 4 fell off the end */
}

void test_shift_is_a_noop_for_degenerate_inputs(void){
    payload_t h[2] = { {7,7}, {8,8} };
    diag_ring_shift(h, sizeof h[0], 1);    /* depth 1: nothing to shift */
    TEST_ASSERT_EQUAL_UINT32(7, h[0].a);
    TEST_ASSERT_EQUAL_UINT32(8, h[1].a);
    diag_ring_shift(h, 0, 2);              /* zero element size */
    TEST_ASSERT_EQUAL_UINT32(8, h[1].a);
    diag_ring_shift(NULL, sizeof h[0], 2); /* must not crash */
}

void test_appendf_accumulates_and_returns_used(void){
    char b[32]; b[0] = '\0';
    size_t u = diag_appendf(b, sizeof b, 0, "ab");
    TEST_ASSERT_EQUAL_UINT32(2, u);
    u = diag_appendf(b, sizeof b, u, "cd%d", 5);
    TEST_ASSERT_EQUAL_UINT32(5, u);
    TEST_ASSERT_EQUAL_STRING("abcd5", b);
}

void test_appendf_truncates_without_overflowing(void){
    char guard[16];
    memset(guard, 0x7E, sizeof guard);
    char *b = guard;                       /* deliberately hand it only 8 of the 16 */
    b[0] = '\0';
    size_t u = diag_appendf(b, 8, 0, "0123456789ABCDEF");
    TEST_ASSERT_EQUAL_UINT32(7, u);                    /* capped at n-1 */
    TEST_ASSERT_EQUAL_STRING("0123456", b);
    TEST_ASSERT_EQUAL_HEX8(0x7E, (unsigned char)guard[8]);  /* byte past n untouched */
}

void test_appendf_past_capacity_is_idempotent(void){
    char b[8]; b[0] = '\0';
    size_t u = diag_appendf(b, sizeof b, 0, "0123456789");
    size_t v = diag_appendf(b, sizeof b, u, "more");
    size_t w = diag_appendf(b, sizeof b, v, "more");
    TEST_ASSERT_EQUAL_UINT32(7, u);
    TEST_ASSERT_EQUAL_UINT32(7, v);
    TEST_ASSERT_EQUAL_UINT32(7, w);
    TEST_ASSERT_EQUAL_STRING("0123456", b);
}

void test_appendf_degenerate_buffers(void){
    char b[1];
    b[0] = 'x';
    TEST_ASSERT_EQUAL_UINT32(0, diag_appendf(b, 1, 0, "hello"));
    TEST_ASSERT_EQUAL_STRING("", b);                   /* terminated, not left as 'x' */
    TEST_ASSERT_EQUAL_UINT32(0, diag_appendf(NULL, 8, 0, "hello"));
    TEST_ASSERT_EQUAL_UINT32(0, diag_appendf(b, 0, 0, "hello"));
}

void test_reset_reason_names(void){
    /* Values confirmed against the installed IDF in Step 1. */
    TEST_ASSERT_EQUAL_STRING("poweron",    diag_reset_reason_name(1));
    TEST_ASSERT_EQUAL_STRING("TASK_WDT",   diag_reset_reason_name(6));
    TEST_ASSERT_EQUAL_STRING("BROWNOUT",   diag_reset_reason_name(9));
    /* Past the plan's table: this IDF's esp_reset_reason_t goes to 15. Losing these to
     * "other" would repeat exactly the mistake the ring exists to prevent -- this board's
     * whole 1.6.x history was watchdog and suspected-power faults. */
    TEST_ASSERT_EQUAL_STRING("efuse",      diag_reset_reason_name(13));
    TEST_ASSERT_EQUAL_STRING("PWR_GLITCH", diag_reset_reason_name(14));
    TEST_ASSERT_EQUAL_STRING("CPU_LOCKUP", diag_reset_reason_name(15));
    TEST_ASSERT_EQUAL_STRING("other",      diag_reset_reason_name(9999));
}

int main(void){
    UNITY_BEGIN();
    RUN_TEST(test_cold_boot_zeroes_and_seeds);
    RUN_TEST(test_warm_boot_bumps_seq_and_keeps_payload);
    RUN_TEST(test_wrong_magic_is_treated_as_cold);
    RUN_TEST(test_shift_moves_history_down_and_leaves_slot_zero);
    RUN_TEST(test_shift_is_a_noop_for_degenerate_inputs);
    RUN_TEST(test_appendf_accumulates_and_returns_used);
    RUN_TEST(test_appendf_truncates_without_overflowing);
    RUN_TEST(test_appendf_past_capacity_is_idempotent);
    RUN_TEST(test_appendf_degenerate_buffers);
    RUN_TEST(test_reset_reason_names);
    return UNITY_END();
}
