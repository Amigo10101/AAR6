//#include "motorcontrol_task.h"
//#include "structs.h"
//#include "constants.h"
//#include "math.h"
//#include "tim.h"
//#include "Motor_control.h"
//#define curve_resolution 51
//
//extern struct MotorStruct motors[NUMBER_OF_JOINTS];
//
//#define TS 0.01
//const double K_p[2] = { 3.1623, 2.7064 };
//const double K_i = 2.2361;
//const double Ad_obs[2][2] = { { 0.9599, 0.0098 }, { -0.0312, 0.9915 } };
//const double Bd_obs[2][2] = { { 0.0003, 0.0401 }, { 0.0098, 0.0312 } };
//float read_measured_position(struct MotorStruct *motor) {
//    // We assume 'motor->position' is the calibrated position in steps
//    // from your encoder, as seen in your struct.
//    return (float)motor->position;
//}
//
//void main_control_loop_for_joint(struct MotorStruct *motor) {
//    float r_cmd;
//    if (motor->commanded_mode == VELOCITY) {
//           // VELOCITY mode: Integrate commanded_velocity to get a ramping target
//           // r_cmd[k+1] = r_cmd[k] + commanded_velocity * Ts
//           motor->lqr_r_cmd = motor->lqr_r_cmd + (motor->commanded_velocity * TS);
//           r_cmd = motor->lqr_r_cmd;
//       } else if (motor->commanded_mode == POSITION) {
//           // POSITION mode: Use commanded_position as the static target
//           r_cmd = (float)motor->commanded_position;
//           // Sync the internal ramp target, so VEL mode starts from here
//           motor->lqr_r_cmd = r_cmd;
//       } else {
//           // STOPPED mode (or any other mode): Hold the last commanded position
//           // Do not integrate or change the target.
//           r_cmd = motor->lqr_r_cmd;
//       }
//    // --- Step 2: Read Inputs ---
//    float y_measured = read_measured_position(motor); // Get actual position (y)
//
//    // --- Step 3: Calculate Error ---
//    float error = r_cmd - y_measured;
//
//    // --- Step 4: Update Integrator State ---
//    // x_i[k+1] = x_i[k] + error * Ts
//    motor->lqr_x_i = motor->lqr_x_i + (error * TS);
//
//    // (Optional: Anti-windup... )
//
//    // --- Step 5: Calculate LQR Acceleration Command (u) ---
//    // The LQR output 'u' is an ACCELERATION command (steps/s^2)
//    // u = -K_p * x_hat - K_i * x_i
//    float u_p = K_p[0] * motor->lqr_x_hat[0] + K_p[1] * motor->lqr_x_hat[1];
//    float u_i = K_i * motor->lqr_x_i;
//    float u_accel = -u_p - u_i; // This is our acceleration command
//
//    // --- Step 6: Integrate Acceleration to get Velocity ---
//    // This is the CRITICAL change.
//    // v[k+1] = v[k] + a[k] * Ts
//    float v_new = motor->lqr_current_velocity + (u_accel * TS);
//
//    // Store the new velocity for the next loop's integration
//    motor->lqr_current_velocity = v_new;
//
//    // --- Step 7: Apply Velocity Command to Motor ---
//    // Send the final velocity (steps/s) to the hardware timer
//    Set_Motor_Velocity(motor, v_new);
//
//    // --- Step 8: Update Observer State for Next Loop ---
//    // The observer MUST use the acceleration 'u' we calculated in  Step 5.
//    // x_hat[k+1] = Ad_obs * x_hat[k] + Bd_obs * [u[k]; y[k]]
//
//    float x_hat_k[2] = {motor->lqr_x_hat[0], motor->lqr_x_hat[1]};
//    float x_hat_new[2];
//
//    x_hat_new[0] = (Ad_obs[0][0] * x_hat_k[0] + Ad_obs[0][1] * x_hat_k[1]) +
//                   (Bd_obs[0][0] * u_accel + Bd_obs[0][1] * y_measured); // Use u_accel
//
//    x_hat_new[1] = (Ad_obs[1][0] * x_hat_k[0] + Ad_obs[1][1] * x_hat_k[1]) +
//                   (Bd_obs[1][0] * u_accel + Bd_obs[1][1] * y_measured); // Use u_accel
//
//    // --- Step 9: Save New State to Struct ---
//    motor->lqr_x_hat[0] = x_hat_new[0];
//    motor->lqr_x_hat[1] = x_hat_new[1];
//}
//
//
//void StartMotorTask(void *argument) {
//    const TickType_t xFrequency = pdMS_TO_TICKS( (TickType_t)(TS * 1000.0f) );
//    TickType_t xLastWakeTime = xTaskGetTickCount();
//	for (;;) {
//		for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
//			 main_control_loop_for_joint(&motors[i]);
//		}
//        vTaskDelayUntil(&xLastWakeTime, xFrequency);
//	}
//}
//
