# parameters.py

# max motor speed for driving sraight ahead or back (PWM value)
FULL_SPD = 60_000

# max motor speed while turning in place (PWM value)
TURN_SPD = 30_000

# half width of "good enough" zone when turning to angle
ANGLE_TOL = 0.035  # radians (2 degrees)

# ratio of spd to joystick value
JS_GAIN = 2

# For turning in place
MAX_ANG_SPD = 0.7
P_TURN_GAIN = 8.0  # Proportional Gain
D_TURN_GAIN = 0.5  # Derivative Gain

# threshold for detectable motion
MIN_DIST = 0.002  # meters
MIN_ANGLE = 0.004  # radians
