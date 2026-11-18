"""Explicit A/B timing for ordinary routes; leave the existing default alone."""
from .route_profiles import point_velocities


def apply_waypoint_timing(trajectory, config, profiles, entry_profiles=()):
    from .local_retiming import timing_mode
    mode = timing_mode(config)
    if mode == 'original':
        return trajectory
    # Reuse the deployed delivery-loop optimizer with the same spatial curve,
    # caps and point identities. Preparation only; runtime sampling is shared.
    from .local_retiming import retime_loaded_leg, timing_dynamics
    limits = config['planning']
    caps = point_velocities(config, profiles)
    entry_caps = point_velocities(config, list(entry_profiles)) if entry_profiles else []
    # Entry points are flown once; each keeps its own timing identity.
    return retime_loaded_leg(trajectory,
        [entry_caps[event['entry_index']] if 'entry_index' in event else caps[event['waypoint_index']]
         for event in trajectory.events],
        [('entry', event['entry_index']) if 'entry_index' in event else ('waypoint', event['waypoint_index'])
         for event in trajectory.events],
        limits['max_velocity_mps'], limits['max_acceleration_mps2'],
        timing_dynamics(config, mode))
