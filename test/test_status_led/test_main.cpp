// Host-side tests for the status LED's priority arbitration. The hardware write
// is replaced with a capture stub, so all of this runs on the laptop.
#include <unity.h>

#include "status_led.h"

using namespace status;

namespace {
Color    g_last{0, 0, 0};
unsigned g_writes = 0;

void capture(uint8_t r, uint8_t g, uint8_t b) {
  g_last = Color{r, g, b};
  ++g_writes;
}

const Pattern kLowInfo{{0, 0, 200}, Priority::kInfo, 0, 100, "low"};
const Pattern kHighErr{{200, 0, 0}, Priority::kError, 0, 100, "high"};
const Pattern kAlsoInfo{{0, 200, 0}, Priority::kInfo, 0, 100, "also-info"};
const Pattern kBlink{{0, 0, 200}, Priority::kInfo, 100, 50, "blink"};

// StatusLed is deliberately non-movable (it owns a lock), so it is initialised
// in place rather than returned by value.
void initLed(StatusLed &l, uint8_t brightness = 100) {
  l.begin(&capture, brightness);
  g_writes = 0;
}
}  // namespace

void setUp(void) {}
void tearDown(void) {}

// begin() must blank the LED: a WS2812 holds power-on garbage otherwise.
static void test_begin_blanks_the_led(void) {
  g_writes = 0;
  StatusLed l;
  l.begin(&capture, 50);
  TEST_ASSERT_EQUAL_UINT8(0, g_last.r);
  TEST_ASSERT_EQUAL_UINT8(0, g_last.g);
  TEST_ASSERT_EQUAL_UINT8(0, g_last.b);
  TEST_ASSERT_TRUE(g_writes >= 1);
}

static void test_brightness_scales_output(void) {
  StatusLed l;
  initLed(l, 10);
  l.claim({{200, 100, 50}, Priority::kInfo, 0, 100, "c"}, 0, 1000);
  l.tick(1000);
  TEST_ASSERT_EQUAL_UINT8(20, l.lastWritten().r);
  TEST_ASSERT_EQUAL_UINT8(10, l.lastWritten().g);
  TEST_ASSERT_EQUAL_UINT8(5, l.lastWritten().b);

  l.setBrightness(50);
  l.tick(1000);
  TEST_ASSERT_EQUAL_UINT8(100, l.lastWritten().r);

  l.setBrightness(200);  // clamped
  TEST_ASSERT_EQUAL_UINT8(100, l.brightness());
}

static void test_higher_priority_wins(void) {
  StatusLed l;
  initLed(l);
  l.claim(kLowInfo, 0, 1000);
  l.tick(1000);
  TEST_ASSERT_EQUAL_UINT8(200, l.lastWritten().b);

  const Token hi = l.claim(kHighErr, 0, 1001);
  l.tick(1001);
  TEST_ASSERT_EQUAL_STRING("high", l.activePattern()->name);
  TEST_ASSERT_EQUAL_UINT8(200, l.lastWritten().r);

  // Releasing the winner must fall back, not go dark.
  l.release(hi);
  l.tick(1002);
  TEST_ASSERT_EQUAL_STRING("low", l.activePattern()->name);
}

// A routine info blink must never paint over an alarm.
static void test_lower_priority_cannot_displace(void) {
  StatusLed l;
  initLed(l);
  l.claim(kHighErr, 0, 1000);
  l.claim(kLowInfo, 0, 1001);
  l.tick(1001);
  TEST_ASSERT_EQUAL_STRING("high", l.activePattern()->name);
}

static void test_equal_priority_newest_wins(void) {
  StatusLed l;
  initLed(l);
  l.claim(kLowInfo, 0, 1000);
  l.claim(kAlsoInfo, 0, 1001);
  l.tick(1001);
  TEST_ASSERT_EQUAL_STRING("also-info", l.activePattern()->name);
}

static void test_timed_claims_expire(void) {
  StatusLed l;
  initLed(l);
  l.claim(kLowInfo, 0, 1000);            // held
  l.claim(kHighErr, 50, 1000);           // momentary
  l.tick(1010);
  TEST_ASSERT_EQUAL_STRING("high", l.activePattern()->name);

  l.tick(1060);  // past the 50 ms hold
  TEST_ASSERT_EQUAL_STRING("low", l.activePattern()->name);
  TEST_ASSERT_EQUAL_UINT(1, l.activeClaims());
}

static void test_blink_follows_duty_cycle(void) {
  StatusLed l;
  initLed(l);
  l.claim(kBlink, 0, 0);
  l.tick(1000);  // phase 0 of a 100 ms period, 50 % duty -> on
  TEST_ASSERT_EQUAL_UINT8(200, l.lastWritten().b);
  l.tick(1060);  // phase 60 -> off
  TEST_ASSERT_EQUAL_UINT8(0, l.lastWritten().b);
  l.tick(1100);  // phase 0 again -> on
  TEST_ASSERT_EQUAL_UINT8(200, l.lastWritten().b);
}

// A token from a released slot must not free whoever reused that slot.
static void test_stale_token_cannot_free_a_reused_slot(void) {
  StatusLed l;
  initLed(l);
  const Token old = l.claim(kLowInfo, 0, 1000);
  l.release(old);
  l.claim(kHighErr, 0, 1001);
  l.release(old);  // stale
  l.tick(1002);
  TEST_ASSERT_EQUAL_STRING("high", l.activePattern()->name);
  TEST_ASSERT_EQUAL_UINT(1, l.activeClaims());
}

static void test_no_claims_means_dark(void) {
  StatusLed l;
  initLed(l);
  const Token t = l.claim(kLowInfo, 0, 1000);
  l.tick(1000);
  l.release(t);
  l.tick(1001);
  TEST_ASSERT_NULL(l.activePattern());
  TEST_ASSERT_EQUAL_UINT8(0, l.lastWritten().b);
}

// Filling the table must not let routine traffic evict an alarm.
static void test_full_table_protects_high_priority(void) {
  StatusLed l;
  initLed(l);
  l.claim(kHighErr, 0, 1000);
  for (size_t i = 0; i < kMaxClaims + 4; ++i) l.claim(kLowInfo, 0, 1001 + i);
  l.tick(2000);
  TEST_ASSERT_EQUAL_STRING("high", l.activePattern()->name);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_begin_blanks_the_led);
  RUN_TEST(test_brightness_scales_output);
  RUN_TEST(test_higher_priority_wins);
  RUN_TEST(test_lower_priority_cannot_displace);
  RUN_TEST(test_equal_priority_newest_wins);
  RUN_TEST(test_timed_claims_expire);
  RUN_TEST(test_blink_follows_duty_cycle);
  RUN_TEST(test_stale_token_cannot_free_a_reused_slot);
  RUN_TEST(test_no_claims_means_dark);
  RUN_TEST(test_full_table_protects_high_priority);
  UNITY_END();
  return 0;
}
