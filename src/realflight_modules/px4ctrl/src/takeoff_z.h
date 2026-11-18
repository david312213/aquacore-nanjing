#ifndef AIRSEAFLY_TAKEOFF_Z_H
#define AIRSEAFLY_TAKEOFF_Z_H

#include <algorithm>
#include <cmath>

// Only the vertical takeoff reference is changed; the upstream controller,
// rotor spin-up and landing trajectory are unchanged. Units: metres, seconds.
namespace airseafly_takeoff
{
struct ZReference
{
    double position;
    double velocity;
};

inline bool valid_ascent(double start_z, double target_z, double speed)
{
    return std::isfinite(start_z) && std::isfinite(target_z) &&
           std::isfinite(speed) && target_z > start_z && speed > 0.0;
}

inline ZReference world_z_reference(double start_z, double target_z,
                                   double speed, double elapsed)
{
    const double z = std::min(target_z, start_z + speed * std::max(0.0, elapsed));
    return {z, (elapsed < 0.0 || z >= target_z) ? 0.0 : speed};
}

inline bool reached_world_z(double measured_z, double reference_z, double target_z)
{
    // After the ramp stops, allow 5 cm tracking tolerance so asymptotic approach
    // to the target does not leave the FSM in AUTO_TAKEOFF forever. AUTO_HOVER
    // still holds the exact target_z, not this tolerance boundary.
    return measured_z >= target_z ||
           (reference_z >= target_z && std::abs(measured_z - target_z) <= 0.05);
}
} // namespace airseafly_takeoff

#endif
