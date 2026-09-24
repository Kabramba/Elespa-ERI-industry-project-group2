#include "can/active_profile.h"

#include "can/can_profile_registry.h"

namespace can {
namespace {
const CanBusProfile *g_active = nullptr;
bool                 g_chosen = false;
}  // namespace

const CanBusProfile &active() {
  if (g_active == nullptr) g_active = profileAt(0);
  return *g_active;
}

bool setActive(const char *key) {
  const CanBusProfile *p = findProfile(key);
  if (p == nullptr) return false;
  g_active = p;
  g_chosen = true;
  return true;
}

bool setActiveIndex(size_t index) {
  const CanBusProfile *p = profileAt(index);
  if (p == nullptr) return false;
  g_active = p;
  g_chosen = true;
  return true;
}

bool activeWasChosen() { return g_chosen; }

}  // namespace can
