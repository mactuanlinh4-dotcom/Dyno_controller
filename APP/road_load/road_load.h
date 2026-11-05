/**
  ******************************************************************************
  * @file           : road_load.h
  * @brief          : Mô phỏng tải đường cho Dyno (Road Load Simulation)
  *                   Fw = m*a + 0.088*m + (0.000015*m + 0.02) * v_kmh^2   [N]
  *                   T_phanh = Fw * r_banh                                  [N.m]
  ******************************************************************************
  */
#ifndef __ROAD_LOAD_H
#define __ROAD_LOAD_H

#include <stdint.h>

/* Giá trị mặc định khi chưa nhập từ PC */
#define ROAD_LOAD_DEFAULT_MASS_KG       150.0f  /* Khối lượng xe + người lái */
#define ROAD_LOAD_DEFAULT_RADIUS_M      0.28f   /* Bán kính bánh xe */

/* Giới hạn nhập liệu hợp lệ */
#define ROAD_LOAD_MASS_MIN_KG           30.0f
#define ROAD_LOAD_MASS_MAX_KG           1000.0f
#define ROAD_LOAD_RADIUS_MIN_M          0.10f
#define ROAD_LOAD_RADIUS_MAX_M          0.60f

/* Hệ số quy đổi vận tốc góc encoder (rad/s) sang vận tốc dài xe (m/s) */
#define ROAD_LOAD_OMEGA_TO_MS           0.111f

/* Mô-men quán tính các chi tiết quay của dyno (kg.m^2), cùng trục encoder:
 *  - Phanh bột từ: 0.0915
 *  - Con lăn (CAD, 18.785 kg): Ixx quy về trọng tâm = 373552 lbm.mm^2 = 0.16944
 * Quy về mặt lăn (bán kính 0.111 m): m_dyno = J_tong / r^2 ≈ 21.18 kg.
 * Phần quán tính này có sẵn về cơ khí nên phanh chỉ cần mô phỏng phần còn
 * thiếu: F_qt = (m - m_dyno) * a  */
#define ROAD_LOAD_PAU_INERTIA_KGM2      0.0915f
#define ROAD_LOAD_ROLLER_INERTIA_KGM2   0.16944f
#define ROAD_LOAD_DYNO_INERTIA_KGM2     (ROAD_LOAD_PAU_INERTIA_KGM2 + ROAD_LOAD_ROLLER_INERTIA_KGM2)
#define ROAD_LOAD_DYNO_MASS_EQ_KG       (ROAD_LOAD_DYNO_INERTIA_KGM2 / \
                                         (ROAD_LOAD_OMEGA_TO_MS * ROAD_LOAD_OMEGA_TO_MS))

/* Lọc EMA gia tốc (đạo hàm vận tốc rất nhiễu) */
#define ROAD_LOAD_ACCEL_EMA_ALPHA       0.10f
/* Dưới tốc độ này coi như xe đứng yên => không phanh */
#define ROAD_LOAD_MIN_SPEED_KMH         1.0f

typedef struct {
    float mass_kg;          // Khối lượng xe m_i (kg)
    float wheel_radius_m;   // Bán kính bánh xe (m)

    float v_ms;             // Vận tốc dài (m/s)
    float v_kmh;            // Vận tốc dài (km/h)
    float wheel_omega_rad_s;// Tốc độ quay bánh xe = v / r (rad/s)
    float wheel_rpm;        // Tốc độ quay bánh xe (RPM)
    float accel_ms2;        // Gia tốc đã lọc (m/s^2)
    float force_n;          // Lực cản đường mục tiêu Fw (N), gồm cả quán tính toàn xe
    float brake_force_n;    // Lực phanh cần tạo = Fw - m_dyno*a (N)
    float torque_nm;        // Mô-men phanh yêu cầu (N.m)

    float prev_v_ms;
    uint8_t initialized;
} RoadLoad_HandleTypeDef;

void  RoadLoad_Init(RoadLoad_HandleTypeDef *rl, float mass_kg, float wheel_radius_m);
void  RoadLoad_SetParams(RoadLoad_HandleTypeDef *rl, float mass_kg, float wheel_radius_m);
void  RoadLoad_Reset(RoadLoad_HandleTypeDef *rl);

/**
 * @brief Tính mô-men phanh yêu cầu theo tải đường
 * @param omega_rad_s: Vận tốc góc bánh xe (rad/s) từ encoder
 * @param dt_s: Chu kỳ gọi hàm (giây)
 * @retval Mô-men phanh yêu cầu (N.m), >= 0
 */
float RoadLoad_Process(RoadLoad_HandleTypeDef *rl, float omega_rad_s, float dt_s);

#endif /* __ROAD_LOAD_H */
