//#include "motorcontrol_task.h"
//#include "structs.h"
//#include "constants.h"
//#include "math.h"
//#include "tim.h"
//#define CURVE_RESOLUTION 51
//
//extern struct MotorStruct motors[NUMBER_OF_JOINTS];
//
//float s_curve[51] = {
//    0.07585818f, 0.0831727f, 0.09112296f, 0.09975049f, 0.10909682f,
//    0.11920292f, 0.13010847f, 0.14185106f, 0.15446527f, 0.16798161f,
//    0.18242552f, 0.19781611f, 0.21416502f, 0.23147522f, 0.24973989f,
//    0.26894142f, 0.2890505f, 0.31002552f, 0.33181223f, 0.35434369f,
//    0.37754067f, 0.40131234f, 0.42555748f, 0.450166f, 0.47502081f,
//    0.5f, 0.52497919f, 0.549834f, 0.57444252f, 0.59868766f,
//    0.62245933f, 0.64565631f, 0.66818777f, 0.68997448f, 0.7109495f,
//    0.73105858f, 0.75026011f, 0.76852478f, 0.78583498f, 0.80218389f,
//    0.81757448f, 0.83201839f, 0.84553473f, 0.85814894f, 0.86989153f,
//    0.88079708f, 0.89090318f, 0.90024951f, 0.90887704f, 0.9168273f,
//    0.92414182f
//};
//
//
//const uint8_t accel = 30	; // 30% of 255
//void calculate_next_arr(struct MotorStruct *m) {
//    if (!m) return;
//
//    int32_t current = m->position;
//    int32_t start   = m->start_position;
//    int32_t end     = m->commanded_position;
//
//    int32_t distance = end - start;
//    if (distance == 0) {
//        // no motion
//        return;
//    }
//
//    int32_t abs_distance = (distance > 0) ? distance : -distance;
//    int32_t steps_from_start = current - start;
//    if (steps_from_start < 0) steps_from_start = -steps_from_start;
//    int32_t steps_to_end = end - current;
//    if (steps_to_end < 0) steps_to_end = -steps_to_end;
//
//    // configure your min/max autoreload values (these were in your original code)
//    const uint16_t arr_max = 180*8; // slower (bigger ARR -> slower frequency)
//    const uint16_t arr_min = 30*8; // faster (smaller ARR -> higher freq)
//
//    // Determine accel region in steps.
//    // Use a percent of distance but clamp to reasonable min/max for small moves.
//    int32_t accel_steps = (abs_distance * accel) / 100; // e.g. 30% of distance
//    const int32_t accel_steps_min = 550*10;    // lower cap so extremely small moves still get some ramping
//    const int32_t accel_steps_max = 1000*10;  // upper cap to avoid huge ramp windows
//    if (accel_steps < accel_steps_min) accel_steps = accel_steps_min;
//    if (accel_steps > accel_steps_max) accel_steps = accel_steps_max;
//    if (accel_steps > (abs_distance / 2)) accel_steps = abs_distance / 2; // don't exceed half the move
//
//    uint16_t computed_arr = arr_min; // default: fastest (constant-velocity zone)
//
//    // Acceleration phase (near start)
//    if (steps_from_start <= accel_steps) {
//        // map steps_from_start [0..accel_steps] -> curve index [0..CURVE_RESOLUTION-1]
//        float t = (accel_steps > 0) ? ((float)steps_from_start / (float)accel_steps) : 1.0f;
//        int idx = (int)roundf(t * (CURVE_RESOLUTION - 1));
//        if (idx < 0) idx = 0;
//        if (idx >= CURVE_RESOLUTION) idx = CURVE_RESOLUTION - 1;
//        float s = s_curve[idx];
//        float arrf = (float)arr_max - ((float)(arr_max - arr_min) * s);
//        computed_arr = (uint16_t)(arrf + 0.5f);
//    }
//    // Deceleration phase (near end)
//    else if (steps_to_end <= accel_steps) {
//        float t = (accel_steps > 0) ? ((float)steps_to_end / (float)accel_steps) : 1.0f;
//        int idx = (int)roundf(t * (CURVE_RESOLUTION - 1));
//        if (idx < 0) idx = 0;
//        if (idx >= CURVE_RESOLUTION) idx = CURVE_RESOLUTION - 1;
//        float s = s_curve[idx];
//        float arrf = (float)arr_max - ((float)(arr_max - arr_min) * s);
//        computed_arr = (uint16_t)(arrf + 0.5f);
//    }
//    else {
//        // Constant-velocity middle of the move
//        computed_arr = arr_min;
//    }
//
//    // safety clampsa
//    if (computed_arr < arr_min) computed_arr = arr_min;
//    if (computed_arr > arr_max) computed_arr = arr_max;
//
//    // tiny smoothing to avoid jumps (exponential moving average)
//    // requires prev_arr_value in struct, initialize to 0 at startup.
//    uint16_t prev = m->arr_value;
//    if (prev == 0) {
//        // first time: take computed value directly
//        m->arr_value = computed_arr;
//    } else {
//        // weight: 75% previous, 25% new => smooth but responsive
//        uint32_t smooth = ( (uint32_t)prev * 3 + (uint32_t)computed_arr ) / 4;
//        m->arr_value = (uint16_t)smooth;
//    }
//}
//void StartMotorTask(void *argument) {
//
//	for (;;) {
//		for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
//			struct MotorStruct *m = &motors[i];
//			if (!m->moving || m->commanded_mode == VELOCITY || m->commanded_mode==STOPPED)
//			    continue;
//			calculate_next_arr(m);
//			__HAL_TIM_SET_AUTORELOAD(m->timer_instance, m->arr_value);
//			__HAL_TIM_SET_COMPARE(m->timer_instance, m->timer_channel,
//					m->arr_value / 2);
//		}
//		osDelay(10);
//	}
//}
//
