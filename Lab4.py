# LEGO slot:0 autostart
from hub import port, sound, button, light_matrix
import runloop, color_sensor, distance_sensor, motor_pair, motor, math, sys

# 1. Hardware Configuration & Constants 
LEFT_MOTOR, RIGHT_MOTOR = port.C, port.D
LINE_SENSOR = port.B
DISTANCE_PORT = port.F

DIR_L, DIR_R = -1, 1
WHEEL_RADIUS, TRACK_WIDTH = 2.82, 11.5

Kp, Kd, BASE_SPEED = 1.2, 2.0, 150 

IDLE_NOT_CALIBRATED = "Idle_not_calibrated"
CALIBRATING = "Calibrating"
IDLE_CALIBRATED = "Idle_calibrated"
LINE_TRACKING_FREE = "Linetracking_free"
LINE_TRACKING_OBSTACLES = "Linetracking_obstacles"
LINE_TRACKING_REVERSE = "Linetracking_reverse"
SEARCH_LINE = "Search_line"
FINISHED = "Finished"


def update_pose(x, y, th, pL, pR):
    cL = motor.relative_position(LEFT_MOTOR) * DIR_L
    cR = motor.relative_position(RIGHT_MOTOR) * DIR_R
    dL = (cL - pL) * (math.pi / 180) * WHEEL_RADIUS
    dR = (cR - pR) * (math.pi / 180) * WHEEL_RADIUS
    dist_centro = (dL + dR) / 2
    delta_th = (dR - dL) / TRACK_WIDTH
    return x + dist_centro * math.cos(th + delta_th/2), y + dist_centro * math.sin(th + delta_th/2), th + delta_th, cL, cR

# ==========================================
# 3. Main Loop (FSM)
# ==========================================
async def main():
    motor_pair.pair(motor_pair.PAIR_1, LEFT_MOTOR, RIGHT_MOTOR)
    state = IDLE_NOT_CALIBRATED
    x_pos, y_pos, theta_rad = 0.0, 0.0, 0.0
    prev_eL = motor.relative_position(LEFT_MOTOR) * DIR_L
    prev_eR = motor.relative_position(RIGHT_MOTOR) * DIR_R
    
    black_val, white_val, threshold = 0, 100, 50
    previous_error = 0
    start_th_cal = 0.0
    search_init = None
    filtered_error = 0
    alpha = 0.6
    filtered_steering = 0
    beta = 0.7

    while state != FINISHED:
        x_pos, y_pos, theta_rad, prev_eL, prev_eR = update_pose(x_pos, y_pos, theta_rad, prev_eL, prev_eR)
        
        dist_obj = distance_sensor.distance(DISTANCE_PORT)
        line_ref = color_sensor.reflection(LINE_SENSOR)
        
        print(f"[{state}] X:{x_pos:.1f} Y:{y_pos:.1f} Th:{math.degrees(theta_rad):.1f} | L:{line_ref} D:{dist_obj}")

        # --- FSM Logic ---
        if state == IDLE_NOT_CALIBRATED:
            light_matrix.show_image(light_matrix.IMAGE_HEART)
            if button.pressed(button.LEFT): # [cite: 8]
                state = CALIBRATING
                start_th_cal, min_r, max_r = theta_rad, line_ref, line_ref
                motor_pair.move(motor_pair.PAIR_1, 100, velocity=100)

        elif state == CALIBRATING:
            if line_ref < min_r: min_r = line_ref
            if line_ref > max_r: max_r = line_ref
            if abs(theta_rad - start_th_cal) >= 2 * math.pi: 
                motor_pair.stop(motor_pair.PAIR_1)
                black_val, white_val = min_r, max_r
                threshold = (black_val + white_val) // 2
                state = IDLE_CALIBRATED
                sound.beep(880, 500)
            
            if button.pressed(button.LEFT):
                state = FINISHED

        elif state == IDLE_CALIBRATED:
            light_matrix.show_image(light_matrix.IMAGE_HAPPY)
            if button.pressed(button.RIGHT): 
                state = LINE_TRACKING_FREE
            if button.pressed(button.LEFT):
                state = FINISHED

        elif state == LINE_TRACKING_FREE:

            if dist_obj < 250 and dist_obj != -1:
                state = LINE_TRACKING_OBSTACLES

            elif line_ref > (white_val - 10):
                state = SEARCH_LINE

            else:
                raw_error = line_ref - threshold

                # --- FILTRO ERROR ---
                filtered_error = (alpha * raw_error) + ((1 - alpha) * filtered_error)

                derivative = filtered_error - previous_error

                steering = (filtered_error * Kp) + (derivative * Kd)

                # --- LIMITADOR ---
                steering = max(min(steering, 100), -100)

                # --- FILTRO STEERING ---
                filtered_steering = (beta * steering) + ((1 - beta) * filtered_steering)

                motor_pair.move(
                    motor_pair.PAIR_1,
                    int(filtered_steering),
                    velocity=BASE_SPEED
                )

                previous_error = filtered_error
            
            if button.pressed(button.LEFT):
                state = FINISHED

        elif state == LINE_TRACKING_OBSTACLES:
            if dist_obj >= 250 and dist_obj != -1: 
                state = LINE_TRACKING_FREE
            elif dist_obj <= 100 and dist_obj != -1: 
                state = LINE_TRACKING_REVERSE
            else:
                # Desaceleración progresiva 
                speed = int((dist_obj - 10) * (BASE_SPEED / 30))
                error = line_ref - threshold
                motor_pair.move(motor_pair.PAIR_1, int(error * Kp), velocity=speed)
            
            if button.pressed(button.LEFT):
                state = FINISHED

        elif state == LINE_TRACKING_REVERSE:
            if dist_obj > 100 or dist_obj == -1:
                state = LINE_TRACKING_FREE
            else:
                motor_pair.move(motor_pair.PAIR_1, 0, velocity=-80)
            
            if button.pressed(button.LEFT):
                state = FINISHED

        elif state == SEARCH_LINE:
            if search_init is None:
                search_init = theta_rad
                search_angle_limit = math.radians(30) # Empezamos con un abanico de 30 grados
                search_direction = 1 # 1 para derecha, -1 para izquierda
                print(f"Buscando línea... Origen: {math.degrees(search_init):.1f}")

            diff_angle = theta_rad - search_init

            motor_pair.move(motor_pair.PAIR_1, 100 * search_direction, velocity=120)

            if (search_direction == 1 and diff_angle >= search_angle_limit) or \
               (search_direction == -1 and diff_angle <= -search_angle_limit):
                
                # Invertir dirección y ampliar el abanico para la siguiente pasada
                search_direction *= -1
                search_angle_limit += math.radians(30) # Ampliar 30 grados más cada vez
                print(f"Ampliando abanico a: {math.degrees(search_angle_limit):.1f}")

            # Si el abanico supera los 180 grados, avanzar un poco y reiniciar (opcional)
            if search_angle_limit > math.pi:
                motor_pair.move(motor_pair.PAIR_1, 0, velocity=80) # Avanzar recto un poco
                search_init = theta_rad # Resetear origen
                search_angle_limit = math.radians(30)

            # CONDICIÓN DE ÉXITO: Línea detectada 
            if line_ref <= black_val + 5:                
                print("¡Línea encontrada! Reanudando tracking...")
                search_init = None
                state = LINE_TRACKING_FREE

            # Salida manual por seguridad
            if button.pressed(button.LEFT):
                state = FINISHED

        await runloop.sleep_ms(20)

    motor_pair.stop(motor_pair.PAIR_1)
    await light_matrix.write("FIN")

runloop.run(main())