// serial_sink.h - human-readable dump of a snapshot.
#pragma once

#include "bms_sink.h"

namespace app {

class SerialSink final : public jbd::IBmsSink {
 public:
  const char *name() const override { return "serial"; }
  void onSnapshot(const jbd::BmsSnapshot &s) override;
};

}  // namespace app
