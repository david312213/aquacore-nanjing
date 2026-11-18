"""Original calibrated DF200 intrinsics and distortion."""
# Verified against the onboard raw 1280x720 df200_calib.yaml on 2026-10-04.
CAMERA_MATRIX = ((655.54919, 0., 602.74071),
                 (0., 657.52527, 363.67025), (0., 0., 1.))
DISTORTION = (-0.015492, 0.020420, 0.002329, -0.002667, 0.)
CALIBRATION_SIZE = (1280., 720.)
