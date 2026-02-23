# LEGO slot:0 autostart
from hub import port, sound, button # type: ignore
import runloop # type: ignore
import color_sensor # type: ignore
import distance_sensor # type: ignore
import motor_pair # type: ignore
import motor # type: ignore
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
WHEEL_RADIUS = 2.82 # cm
TRACK_WIDTH = 11.5 # cm
# ==========================================
# 2. Odometry Core (Incremental Logic)
# ==========================================
def update_pose(x_i, y_i, theta_i_rad, prev_eL, prev_eR):

    """
    FUNCTION update_pose:
    // 1. GET NEW SENSOR DATA
    READ current degrees from Left Motor and Right Motor
    // 2. CALCULATE WHEEL MOVEMENT
    COMPUTE how many centimeters the Left Wheel moved since last
    check
    COMPUTE how many centimeters the Right Wheel moved since last
    check
    // 3. CALCULATE ROBOT MOVEMENT
    // Forward movement
    // Turning
    // 4. UPDATE GLOBAL POSITION (X, Y, Angle)
    UPDATE the X coordinate (using Forward_Distance and current
    angle)
    UPDATE the Y coordinate (using Forward_Distance and current
    angle)
    UPDATE the Robot's Angle (adding the Change_in_Angle)
    // 5. FINISH
    RETURN the new X, Y, Angle, and the current motor degrees for the
    next loop

    return x_i, y_i, theta_i_rad, curr_eL, curr_eR
    """

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
    
    # Read the initial encoder values (accounting for motor mounting direction).
    # We need these baselines to calculate how much the wheels turn in the first loop.
    
    prev_eL = motor.relative_position(LEFT_MOTOR) * DIR_L
    prev_eR = motor.relative_position(RIGHT_MOTOR) * DIR_R
    
    
    # ---------------------------------------------------------
    # PHASE 1: Move Forward 10cm
    # ---------------------------------------------------------
    # Start moving the robot. Steering = 0 means go perfectly straight.
    # Velocity = 150 is a moderate speed.
    
    motor_pair.move(motor_pair.PAIR_1, 0, velocity=150)
    
    # Keep looping and updating position AS LONG AS the X coordinate is less than 10 cm.
    while x_pos < 10.0:
        # 1. Ask the math function to calculate our new position based on wheel movement
        
        x_pos, y_pos, theta_rad, prev_eL, prev_eR = update_pose(x_pos, y_pos, theta_rad, prev_eL, prev_eR)
        
        # 2. Convert the internal math angle (Radians) to human-readable Degrees for the graph
        
        t_deg = round(theta_rad * 180 / math.pi, 2)
        
        # 3. Print the data formatted specifically for the VS Code Blocklypy Plotter.
        # # It must start with "plot:" followed by Key=Value pairs separated by commas.
        
        print("plot: X=" + str(round(x_pos, 2)) +
              ", Y=" + str(round(y_pos, 2)) +
              ", Theta=" + str(t_deg) +
              ", Light=" + str(color_sensor.reflection(SENSOR_PORT)) +
              ", Dist=" +
              str(distance_sensor.distance(DISTANCE_PORT)))
        
        # 4. Pause the loop for 20 milliseconds.
        # # This prevents the Hub from freezing and matches the sensor update rates.
        
        await runloop.sleep_ms(20) 
        
    # Stop the motors once we cross the 10cm mark, and wait half a second to stabilize.
    motor_pair.stop(motor_pair.PAIR_1)
    await runloop.sleep_ms(500)


    # ---------------------------------------------------------
    # PHASE 2: Spin in place for 720 degrees (Two full turns)
    # ---------------------------------------------------------
    # We need to know exactly where the encoder is RIGHT NOW before we start spinning,
    # so we can track how much it changes during the spin.
    
    start_spin_encoder = motor.relative_position(LEFT_MOTOR) * DIR_L
    
    # Steering = 100 means a sharp, in-place turn to the right (Left 
    # wheel forward, Right wheel back).
    
    motor_pair.move(motor_pair.PAIR_1, 100, velocity=150)
    
    # We use an infinite loop (while True) and will "break" out of it 
    # manually when done.
    
    while True:
        # Continually update the position so the graph keeps drawing our location
        x_pos, y_pos, theta_rad, prev_eL, prev_eR = update_pose(x_pos, y_pos, theta_rad, prev_eL, prev_eR)
        t_deg = round(theta_rad * 180 / math.pi, 2)
        print("plot: X=" + str(round(x_pos, 2)) +
        ", Y=" + str(round(y_pos, 2)) +
        ", Theta=" + str(t_deg) +
        ", Light=" + str(color_sensor.reflection(SENSOR_PORT)) +
        ", Dist=" +
        str(distance_sensor.distance(DISTANCE_PORT)))
        
        # Calculate how many degrees the left wheel has turned since the spin started.
        # If it reaches 1468 degrees (our calculated target for a 720-degree body spin), exit the loop.
        
        current_encoder = motor.relative_position(LEFT_MOTOR) * DIR_L
        if abs(current_encoder - start_spin_encoder) >= 1468: 
            break # Exit the while loop
        
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