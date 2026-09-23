// ring_sink.h - publishes each snapshot into the shared telemetry ring.
#pragma once

#include "bms_sink.h"
#include "telemetry.h"

namespace app {

class RingSink final : public jbd::IBmsSink {
 public:
  explicit RingSink(TelemetryRing &ring) : ring_(ring) {}
  const char *name() const override { return "ring"; }
  void onSnapshot(const jbd::BmsSnapshot &s) override { ring_.publish(s); }

 private:
  TelemetryRing &ring_;
};

}  // namespace app
