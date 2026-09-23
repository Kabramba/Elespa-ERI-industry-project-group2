// Host-side tests for the IPC ring: overflow semantics, ordering, and both
// read paths. Runs on the laptop, no hardware.
#include <unity.h>

#include "bms_ring.h"

using namespace jbd;

void setUp(void) {}
void tearDown(void) {}

static BmsSnapshot make(uint32_t tag) {
  BmsSnapshot s = {};
  s.uptimeMs  = tag;
  s.packVolts = static_cast<float>(tag);
  s.valid     = true;
  return s;
}

static void test_empty_ring_reports_nothing(void) {
  BmsRing<4> r;
  BmsSnapshot out;
  TEST_ASSERT_FALSE(r.latest(out));
  TEST_ASSERT_NULL(r.latestRef());
  TEST_ASSERT_EQUAL_UINT(0, r.count());
  TEST_ASSERT_EQUAL_UINT32(0, r.publishedCount());
  TEST_ASSERT_EQUAL_UINT32(0, r.overwrittenCount());
  TEST_ASSERT_EQUAL_UINT(4, r.capacity());
}

static void test_latest_returns_newest(void) {
  BmsRing<4> r;
  r.publish(make(1));
  r.publish(make(2));

  BmsSnapshot out;
  TEST_ASSERT_TRUE(r.latest(out));
  TEST_ASSERT_EQUAL_UINT32(2, out.uptimeMs);
  TEST_ASSERT_EQUAL_UINT(2, r.count());
}

static void test_zero_copy_ref_tracks_newest(void) {
  BmsRing<4> r;
  r.publish(make(10));
  const BmsSnapshot *p = r.latestRef();
  TEST_ASSERT_NOT_NULL(p);
  TEST_ASSERT_EQUAL_UINT32(10, p->uptimeMs);

  r.publish(make(11));
  p = r.latestRef();
  TEST_ASSERT_NOT_NULL(p);
  TEST_ASSERT_EQUAL_UINT32(11, p->uptimeMs);

  // Double buffered: consecutive publishes must not land on the same address,
  // otherwise a reader could see a half-written struct.
  const BmsSnapshot *a = r.latestRef();
  r.publish(make(12));
  const BmsSnapshot *b = r.latestRef();
  TEST_ASSERT_TRUE(a != b);
}

// Newest wins: the oldest entry is discarded and the drop is counted.
static void test_overflow_discards_oldest(void) {
  BmsRing<4> r;
  for (uint32_t i = 1; i <= 6; ++i) r.publish(make(i));

  TEST_ASSERT_EQUAL_UINT(4, r.count());
  TEST_ASSERT_EQUAL_UINT32(6, r.publishedCount());
  TEST_ASSERT_EQUAL_UINT32(2, r.overwrittenCount());

  BmsSnapshot out;
  TEST_ASSERT_TRUE(r.latest(out));
  TEST_ASSERT_EQUAL_UINT32(6, out.uptimeMs);
}

static void test_history_is_oldest_first(void) {
  BmsRing<4> r;
  for (uint32_t i = 1; i <= 6; ++i) r.publish(make(i));

  BmsSnapshot buf[8] = {};
  const size_t n = r.history(buf, 8);
  TEST_ASSERT_EQUAL_UINT(4, n);
  TEST_ASSERT_EQUAL_UINT32(3, buf[0].uptimeMs);   // 1 and 2 were dropped
  TEST_ASSERT_EQUAL_UINT32(4, buf[1].uptimeMs);
  TEST_ASSERT_EQUAL_UINT32(5, buf[2].uptimeMs);
  TEST_ASSERT_EQUAL_UINT32(6, buf[3].uptimeMs);
}

static void test_history_respects_max(void) {
  BmsRing<8> r;
  for (uint32_t i = 1; i <= 5; ++i) r.publish(make(i));

  BmsSnapshot buf[2] = {};
  const size_t n = r.history(buf, 2);
  TEST_ASSERT_EQUAL_UINT(2, n);
  TEST_ASSERT_EQUAL_UINT32(4, buf[0].uptimeMs);   // the newest two, oldest first
  TEST_ASSERT_EQUAL_UINT32(5, buf[1].uptimeMs);

  TEST_ASSERT_EQUAL_UINT(0, r.history(nullptr, 4));
  TEST_ASSERT_EQUAL_UINT(0, r.history(buf, 0));
}

static void test_clear_wipes_everything(void) {
  BmsRing<4> r;
  for (uint32_t i = 1; i <= 5; ++i) r.publish(make(i));
  r.clear();

  BmsSnapshot out;
  TEST_ASSERT_FALSE(r.latest(out));
  TEST_ASSERT_NULL(r.latestRef());
  TEST_ASSERT_EQUAL_UINT(0, r.count());
  TEST_ASSERT_EQUAL_UINT32(0, r.publishedCount());
  TEST_ASSERT_EQUAL_UINT32(0, r.overwrittenCount());
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_empty_ring_reports_nothing);
  RUN_TEST(test_latest_returns_newest);
  RUN_TEST(test_zero_copy_ref_tracks_newest);
  RUN_TEST(test_overflow_discards_oldest);
  RUN_TEST(test_history_is_oldest_first);
  RUN_TEST(test_history_respects_max);
  RUN_TEST(test_clear_wipes_everything);
  return UNITY_END();
}
