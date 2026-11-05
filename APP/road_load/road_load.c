#include "road_load.h"
#include <math.h>

static float Clamp(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void RoadLoad_Init(RoadLoad_HandleTypeDef *rl, float mass_kg, float wheel_radius_m) {
    RoadLoad_SetParams(rl, mass_kg, wheel_radius_m);
    RoadLoad_Reset(rl);
}

void RoadLoad_SetParams(RoadLoad_HandleTypeDef *rl, float mass_kg, float wheel_radius_m) {
    rl->mass_kg        = Clamp(mass_kg, ROAD_LOAD_MASS_MIN_KG, ROAD_LOAD_MASS_MAX_KG);
    rl->wheel_radius_m = Clamp(wheel_radius_m, ROAD_LOAD_RADIUS_MIN_M, ROAD_LOAD_RADIUS_MAX_M);
}

void RoadLoad_Reset(RoadLoad_HandleTypeDef *rl) {
    rl->v_ms        = 0.0f;
    rl->v_kmh       = 0.0f;
    rl->wheel_omega_rad_s = 0.0f;
    rl->wheel_rpm   = 0.0f;
    rl->accel_ms2   = 0.0f;
    rl->force_n     = 0.0f;
    rl->brake_force_n = 0.0f;
    rl->torque_nm   = 0.0f;
    rl->prev_v_ms   = 0.0f;
    rl->initialized = 0;
}

float RoadLoad_Process(RoadLoad_HandleTypeDef *rl, float omega_rad_s, float dt_s) {
    if (dt_s <= 0.0f) {
        return rl->torque_nm;
    }

    /* Vận tốc dài của xe: v (m/s) = omega_encoder (rad/s) * 0.111 */
    rl->v_ms  = fabsf(omega_rad_s) * ROAD_LOAD_OMEGA_TO_MS;
    rl->v_kmh = rl->v_ms * 3.6f;

    /* Tốc độ quay bánh xe: omega_banh = v / r (r do người dùng nhập) */
    rl->wheel_omega_rad_s = rl->v_ms / rl->wheel_radius_m;
    rl->wheel_rpm         = rl->wheel_omega_rad_s * (30.0f / 3.14159265f);

    /* Gia tốc a = dv/dt, lọc EMA. Lần đầu chưa có mẫu trước => a = 0 */
    float a_raw = 0.0f;
    if (rl->initialized) {
        a_raw = (rl->v_ms - rl->prev_v_ms) / dt_s;
    }
    rl->prev_v_ms   = rl->v_ms;
    rl->initialized = 1;
    rl->accel_ms2 = (ROAD_LOAD_ACCEL_EMA_ALPHA * a_raw) +
                    ((1.0f - ROAD_LOAD_ACCEL_EMA_ALPHA) * rl->accel_ms2);

    /* Xe đứng yên => nhả phanh để không khóa bánh khi khởi hành */
    if (rl->v_kmh < ROAD_LOAD_MIN_SPEED_KMH) {
        rl->force_n   = 0.0f;
        rl->brake_force_n = 0.0f;
        rl->torque_nm = 0.0f;
        return 0.0f;
    }

    /* Fw = m*a + 0.088*m + (0.000015*m + 0.02) * v_kmh^2 */
    float m = rl->mass_kg;
    float fw = (m * rl->accel_ms2)
             + (0.088f * m)
             + ((0.000015f * m + 0.02f) * rl->v_kmh * rl->v_kmh);
    rl->force_n = fw;

    /* Trừ phần quán tính cơ khí sẵn có (phanh bột + con lăn): F_phanh = Fw - m_dyno*a */
    float f_brake = fw - (ROAD_LOAD_DYNO_MASS_EQ_KG * rl->accel_ms2);

    /* Phanh bột chỉ tạo lực cản, không đẩy được (khi xe giảm tốc mạnh F < 0) */
    if (f_brake < 0.0f) f_brake = 0.0f;
    rl->brake_force_n = f_brake;

    /* Lực tác dụng tại mặt lăn bán kính 0.111 m => mô-men trên trục phanh */
    rl->torque_nm = f_brake * ROAD_LOAD_OMEGA_TO_MS;
    return rl->torque_nm;
}
