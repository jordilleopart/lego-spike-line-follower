# LEGO slot:0 autostart
from hub import port, sound, button  # type: ignore
import runloop  # type: ignore
import color_sensor  # type: ignore
import distance_sensor  # type: ignore
import motor_pair  # type: ignore
import motor  # type: ignore
import math

# ==========================================
# 1. Hardware Configuration & Constants
# ==========================================
LEFT_MOTOR = port.C
RIGHT_MOTOR = port.D
SENSOR_PORT = port.B
DISTANCE_PORT = port.F
DIR_L = -1
DIR_R = 1
WHEEL_RADIUS = 2.82  # cm
TRACK_WIDTH = 11.5  # cm

# State Definitions
WAIT = "Wait"
STRAIGHT = "StraightMovement"
ROTATION = "RotationMovement"

# ==========================================
# 2. Odometry Core (Incremental Logic)
# ==========================================
def update_pose(x_i, y_i, theta_i_rad, prev_eL, prev_eR):
    """
    FUNCTION update_pose:
    // 1. GET NEW SENSOR DATA
    READ current degrees from Left Motor and Right Motor
    // 2. CALCULATE WHEEL MOVEMENT
    COMPUTE how many centimeters the Left Wheel moved since last check
    COMPUTE how many centimeters the Right Wheel moved since last check
    // 3. CALCULATE ROBOT MOVEMENT
    // Forward movement
    // Turning
    // 4. UPDATE GLOBAL POSITION (X, Y, Angle)
    UPDATE the X coordinate (using Forward_Distance and current angle)
    UPDATE the Y coordinate (using Forward_Distance and current angle)
    UPDATE the Robot's Angle (adding the Change_in_Angle)
    // 5. FINISH
    RETURN the new X, Y, Angle, and the current motor degrees for the next loop
    """
    # Get current encoder values
    curr_eL = motor.relative_position(LEFT_MOTOR) * DIR_L
    curr_eR = motor.relative_position(RIGHT_MOTOR) * DIR_R

    # Calculate wheel distances (degrees to cm)
    delta_L = (curr_eL - prev_eL) * (math.pi / 180) * WHEEL_RADIUS
    delta_R = (curr_eR - prev_eR) * (math.pi / 180) * WHEEL_RADIUS

    # Calculate forward movement and rotation
    forward_dist = (delta_L + delta_R) / 2
    delta_theta = (delta_R - delta_L) / TRACK_WIDTH

    # Update pose
    x_i += forward_dist * math.cos(theta_i_rad + delta_theta / 2)
    y_i += forward_dist * math.sin(theta_i_rad + delta_theta / 2)
    theta_i_rad += delta_theta

    return x_i, y_i, theta_i_rad, curr_eL, curr_eR

# ==========================================
# 3. Main Routine
# ==========================================
async def main():
    # ---------------------------------------------------------
    # INITIALIZATION
    # ---------------------------------------------------------
    # Pair the left and right motors so they can be driven together as a single base
    motor_pair.pair(motor_pair.PAIR_1, LEFT_MOTOR, RIGHT_MOTOR)

    # Set the robot's starting position (X, Y) and angle (Theta) to zero.
    # This is the origin point (0,0) on our invisible map.
    x_pos, y_pos, theta_rad = 0.0, 0.0, 0.0
    state = WAIT
    start_theta_rad = 0.0  # Track rotation start angle

    # Read the initial encoder values (accounting for motor mounting direction).
    # We need these baselines to calculate how much the wheels turn in the first loop.
    prev_eL = motor.relative_position(LEFT_MOTOR) * DIR_L
    prev_eR = motor.relative_position(RIGHT_MOTOR) * DIR_R

    while True:
        if state == WAIT:
            if button.pressed(button.LEFT):
                state = STRAIGHT
                print("State: STRAIGHT_MOVEMENT")
        
        elif state == STRAIGHT:
            motor_pair.move(motor_pair.PAIR_1, 0, velocity=150)
            x_pos, y_pos, theta_rad, prev_eL, prev_eR = \
                update_pose(x_pos, y_pos, theta_rad, prev_eL, prev_eR)
            if x_pos >= 20.0:
                state = ROTATION
                motor_pair.stop(motor_pair.PAIR_1)
                start_theta_rad = theta_rad  # Save angle when rotation starts
                print("State: ROTATION")
        
        elif state == ROTATION:
            # Rotation logic - max 360 degrees
            motor_pair.move(motor_pair.PAIR_1, 90, velocity=150)
            x_pos, y_pos, theta_rad, prev_eL, prev_eR = \
                update_pose(x_pos, y_pos, theta_rad, prev_eL, prev_eR)
            
            # Check if rotated 360 degrees (2*pi radians)
            if abs(theta_rad - start_theta_rad) >= 2 * math.pi:
                motor_pair.stop(motor_pair.PAIR_1)
                state = WAIT
                print("State: WAIT (rotation complete)")
        
        await runloop.sleep_ms(20)

    # ---------------------------------------------------------
    # PHASE 3: Finish and Cleanup
    # ---------------------------------------------------------
    # Stop the motors completely
    motor_pair.stop(motor_pair.PAIR_1)

    # Play a success beep (Frequency=440Hz / A4 note, Duration=500ms, Volume=100%)
    sound.beep(440, 500, 100)

# Start the asynchronous event loop to run the main function
runloop.run(main())
