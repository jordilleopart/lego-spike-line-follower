# LEGO slot:0 autostart
from hub import port, sound, button, light_matrix
import runloop, color_sensor, distance_sensor, motor_pair, motor, force_sensor, math

# ==========================================
# 1. Hardware Configuration & Constants
# ==========================================
LEFT_MOTOR, RIGHT_MOTOR = port.C, port.D
LINE_SENSOR = port.B
DISTANCE_PORT = port.F
FORCE_SENSOR_PORT = port.A  

DIR_L, DIR_R = -1, 1
WHEEL_RADIUS, TRACK_WIDTH = 2.82, 11.5

# PD controller parameters (tuned for stability)
Kp, Kd, BASE_SPEED = 1.0, 1.5, 120

# ==========================================
# 2. FSM State Definitions
# ==========================================
IDLE_NOT_CALIBRATED = "Idle_not_calibrated"
CALIBRATING = "Calibrating"
IDLE_CALIBRATED = "Idle_calibrated"
LINE_TRACKING_FREE = "Linetracking_free"
LINE_TRACKING_OBSTACLES = "Linetracking_obstacles"
LINE_TRACKING_REVERSE = "Linetracking_reverse"
BLOCKED = "Blocked"
SEARCH_LINE = "Search_line"
FINISHED = "Finished"

# ==========================================
# 3. Odometry Update Function
# ==========================================
def update_pose(x, y, th, pL, pR):
    cL = motor.relative_position(LEFT_MOTOR) * DIR_L
    cR = motor.relative_position(RIGHT_MOTOR) * DIR_R

    dL = (cL - pL) * (math.pi / 180) * WHEEL_RADIUS
    dR = (cR - pR) * (math.pi / 180) * WHEEL_RADIUS

    dist_center = (dL + dR) / 2
    delta_th = (dR - dL) / TRACK_WIDTH

    return (
        x + dist_center * math.cos(th + delta_th / 2),
        y + dist_center * math.sin(th + delta_th / 2),
        th + delta_th,
        cL,
        cR
    )

# ==========================================
# 4. Main FSM Loop
# ==========================================
async def main():
    motor_pair.pair(motor_pair.PAIR_1, LEFT_MOTOR, RIGHT_MOTOR)

    state = IDLE_NOT_CALIBRATED

    x_pos, y_pos, theta_rad = 0.0, 0.0, 0.0
    prev_eL = motor.relative_position(LEFT_MOTOR) * DIR_L
    prev_eR = motor.relative_position(RIGHT_MOTOR) * DIR_R

    black_val, white_val, threshold = 0, 100, 50

    previous_error = 0
    filtered_error = 0
    filtered_steering = 0

    alpha = 0.6   # Error filter coefficient
    beta = 0.7    # Steering filter coefficient

    # Line loss handling
    line_lost_counter = 0
    LINE_LOST_THRESHOLD = 5
    last_valid_error = 0

    search_init = None

    while state != FINISHED:
        # --- Odometry update ---
        x_pos, y_pos, theta_rad, prev_eL, prev_eR = update_pose(
            x_pos, y_pos, theta_rad, prev_eL, prev_eR
        )

        # --- Sensor readings ---
        dist_obj = distance_sensor.distance(DISTANCE_PORT)
        line_ref = color_sensor.reflection(LINE_SENSOR)
        rear_pressed = force_sensor.pressed(FORCE_SENSOR_PORT)

        print(f"[{state}] X:{x_pos:.1f} Y:{y_pos:.1f} Th:{math.degrees(theta_rad):.1f} | L:{line_ref} D:{dist_obj}")

        # ==========================================
        # STATE: IDLE_NOT_CALIBRATED
        # ==========================================
        if state == IDLE_NOT_CALIBRATED:
            light_matrix.show_image(light_matrix.IMAGE_HEART)

            if button.pressed(button.LEFT):
                state = CALIBRATING
                start_th_cal = theta_rad
                min_r, max_r = line_ref, line_ref

                motor_pair.move(motor_pair.PAIR_1, 100, velocity=100)

        # ==========================================
        # STATE: CALIBRATING
        # ==========================================
        elif state == CALIBRATING:
            if line_ref < min_r:
                min_r = line_ref
            if line_ref > max_r:
                max_r = line_ref

            # Detect full rotation (~360°)
            if abs(theta_rad - start_th_cal) >= 2 * math.pi:
                motor_pair.stop(motor_pair.PAIR_1)

                black_val, white_val = min_r, max_r
                threshold = (black_val + white_val) // 2

                state = IDLE_CALIBRATED
                sound.beep(880, 500)

        # ==========================================
        # STATE: IDLE_CALIBRATED
        # ==========================================
        elif state == IDLE_CALIBRATED:
            light_matrix.show_image(light_matrix.IMAGE_HAPPY)

            if button.pressed(button.RIGHT):
                state = LINE_TRACKING_FREE

        # ==========================================
        # STATE: LINE_TRACKING_FREE
        # ==========================================
        elif state == LINE_TRACKING_FREE:
            light_matrix.show_image(light_matrix.IMAGE_HAPPY)

            # --- Obstacle detection ---
            if dist_obj < 250 and dist_obj != -1:
                state = LINE_TRACKING_OBSTACLES
                line_lost_counter = 0

            # --- Gradual line loss detection ---
            elif line_ref > white_val - 10:
                line_lost_counter += 1

                if line_lost_counter >= LINE_LOST_THRESHOLD:
                    state = SEARCH_LINE
                    line_lost_counter = 0
                else:
                    # --- Short-term recovery ---
                    recovery_steering = 60 if last_valid_error >= 0 else -60
                    motor_pair.move(motor_pair.PAIR_1, recovery_steering, velocity=BASE_SPEED // 2)

            else:
                # --- Normal tracking ---
                line_lost_counter = 0

                raw_error = line_ref - threshold

                # --- LOW-PASS FILTER (Error) ---
                filtered_error = alpha * raw_error + (1 - alpha) * filtered_error

                derivative = filtered_error - previous_error

                # --- PD CONTROLLER ---
                steering = filtered_error * Kp + derivative * Kd

                # --- LIMIT STEERING ---
                steering = max(min(steering, 100), -100)

                # --- STEERING FILTER ---
                filtered_steering = beta * steering + (1 - beta) * filtered_steering

                # --- ADAPTIVE SPEED ---
                speed = BASE_SPEED - int(abs(filtered_steering) * 0.5)
                speed = max(speed, 60)

                motor_pair.move(
                    motor_pair.PAIR_1,
                    int(filtered_steering),
                    velocity=speed
                )

                previous_error = filtered_error
                last_valid_error = filtered_error

        # ==========================================
        # STATE: LINE_TRACKING_OBSTACLES
        # ==========================================
        elif state == LINE_TRACKING_OBSTACLES:
            if dist_obj >= 250 or dist_obj == -1:
                state = LINE_TRACKING_FREE

            elif dist_obj <= 100:
                state = LINE_TRACKING_REVERSE

            else:
                # --- Progressive deceleration ---
                min_speed = 30
                speed = int(min_speed + (dist_obj - 100) * (BASE_SPEED - min_speed) / 150)

                error = line_ref - threshold

                motor_pair.move(
                    motor_pair.PAIR_1,
                    int(error * Kp),
                    velocity=speed
                )

        # ==========================================
        # STATE: LINE_TRACKING_REVERSE
        # ==========================================
        elif state == LINE_TRACKING_REVERSE:
            if rear_pressed:
                motor_pair.stop(motor_pair.PAIR_1)
                sound.beep(440, 200)
                state = BLOCKED

            elif dist_obj > 100 or dist_obj == -1:
                state = LINE_TRACKING_FREE

            else:
                error = line_ref - threshold

                motor_pair.move(
                    motor_pair.PAIR_1,
                    int(-error * Kp),
                    velocity=-60
                )

        # ==========================================
        # STATE: BLOCKED
        # ==========================================
        elif state == BLOCKED:
            motor_pair.stop(motor_pair.PAIR_1)
            light_matrix.show_image(light_matrix.IMAGE_SAD)

            front_clear = (dist_obj > 150 or dist_obj == -1)
            rear_clear = not rear_pressed

            if front_clear:
                sound.beep(880, 200)
                light_matrix.show_image(light_matrix.IMAGE_HAPPY)
                state = LINE_TRACKING_FREE

            elif rear_clear:
                state = LINE_TRACKING_REVERSE

        # ==========================================
        # STATE: SEARCH_LINE
        # ==========================================
        elif state == SEARCH_LINE:
            if search_init is None:
                search_init = theta_rad
                search_direction = 1 if previous_error >= 0 else -1
                search_angle_limit = math.radians(35)

            diff_angle = theta_rad - search_init

            motor_pair.move(motor_pair.PAIR_1, 80 * search_direction, velocity=110)

            # --- Expand search arc ---
            if (search_direction == 1 and diff_angle >= search_angle_limit) or \
               (search_direction == -1 and diff_angle <= -search_angle_limit):

                search_direction *= -1
                search_angle_limit += math.radians(25)

                motor_pair.move(motor_pair.PAIR_1, 0, velocity=100)
                await runloop.sleep_ms(80)

            # --- Line reacquired ---
            if line_ref <= black_val + 5:
                motor_pair.stop(motor_pair.PAIR_1)

                search_init = None
                filtered_error = 0
                filtered_steering = 0
                previous_error = 0

                state = LINE_TRACKING_FREE

            # --- Fallback if search too large ---
            if search_angle_limit > math.radians(200):
                motor_pair.move(motor_pair.PAIR_1, 0, velocity=120)
                await runloop.sleep_ms(300)
                search_init = theta_rad
                search_angle_limit = math.radians(35)

            if button.pressed(button.LEFT):
                state = FINISHED

        await runloop.sleep_ms(20)

    # ==========================================
    # STATE: FINISHED
    # ==========================================
    motor_pair.stop(motor_pair.PAIR_1)
    await light_matrix.write("END")

runloop.run(main())