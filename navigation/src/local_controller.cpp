#include "parkour_navigation/local_controller.h"
#include <cmath>
#include <algorithm>

double StanleyController::ComputeSteering(double e_y, double e_psi, double v_t) {
    // 限制最小速度以避免除以零的奇点
    double v_safe = std::max(v_t, 0.1);

    // 计算非线性横向误差反馈角
    // arctan(k * e_y / v_t)
    double cross_track_steering = atan2(params_.k_gain * e_y, v_safe);

    // 综合航向误差与横向纠偏
    // \delta(t) = e_\psi(t) + arctan(k * e_y(t) / v_t)
    double steer_angle = e_psi + cross_track_steering;

    // 限制最大打方向盘的角度 (车辆运动学约束)
    steer_angle = std::clamp(steer_angle, -params_.max_steer_angle, params_.max_steer_angle);

    return steer_angle;
}