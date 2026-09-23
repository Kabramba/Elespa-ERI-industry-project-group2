// bms_sink.h - a consumer of pack telemetry.
//
// The snapshot arrives by const reference: a sink can read it and forward it,
// but cannot alter what any other sink will see.
#pragma once

#include "bms_types.h"

namespace jbd {

class IBmsSink {
 public:
  virtual ~IBmsSink() = default;
  virtual const char *name() const = 0;
  virtual void onSnapshot(const BmsSnapshot &s) = 0;
};

}  // namespace jbd
