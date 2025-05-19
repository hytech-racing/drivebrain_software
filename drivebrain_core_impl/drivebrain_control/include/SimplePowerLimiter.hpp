#include <VehicleDataTypes.hpp>

namespace control {
namespace util {
core::SpeedControlOut apply_power_limit(core::SpeedControlOut current_control,
                                        veh_vec<float> current_rpms, float max_power_kw);
}

} // namespace control
