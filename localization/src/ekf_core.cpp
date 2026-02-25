#include "parkour_localization/ekf_core.h"
#include <cmath>

void SemanticEKF::Predict(double dt, double a_t, double e_psi) {
    // 1. 状态预测: x_t = [s_t, v_t]^T 
    // s_t = s_{t-1} + (v_{t-1}*dt + 0.5*a_t*dt^2) * cos(e_psi)
    double ds = (state_(1) * dt + 0.5 * a_t * dt * dt) * cos(e_psi);
    state_(0) += ds;
    state_(1) += a_t * dt;

    // 2. 雅可比矩阵 F_t 计算
    Eigen::Matrix2d F_t;
    F_t << 1.0, dt * cos(e_psi),
           0.0, 1.0;

    // 3. 协方差预测: P_t = F_t * P_{t-1} * F_t^T + Q_t
    P_ = F_t * P_ * F_t.transpose() + Q_;
}

void SemanticEKF::UpdateWithSemanticObservation(double s_map, double u_bbox, double e_psi, double D_lat, double conf_yolo) {
    // 1. 计算几何补偿 (Geometric Compensation)
    // \Delta s_{comp} = D_{lat} * ((u_bbox - u_c) / f_x - tan(e_psi))
    double delta_s_comp = D_lat * ((u_bbox - params_.u_c) / params_.f_x - tan(e_psi));
    
    // 2. 计算精细化观测值 z_t
    double z_t = s_map - params_.L_cam + delta_s_comp;

    // 3. 动态协方差调整 (Dynamic Covariance Scaling)
    // R_t = R_0 + w_1*(1 - P_yolo) + ... (此处简化展示)
    double R_t = params_.R_0 + params_.w1 * (1.0 - conf_yolo) + params_.w3 * std::abs(e_psi);
    Eigen::Matrix<double, 1, 1> R;
    R << R_t;

    // 4. 标准 EKF 更新步骤
    Eigen::Matrix<double, 1, 2> H;
    H << 1.0, 0.0;
    
    Eigen::Matrix<double, 1, 1> S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, 2, 1> K = P_ * H.transpose() * S.inverse();
    
    double y_tilde = z_t - (H * state_)(0, 0); // 创新值 (Innovation)
    
    // 异常值剔除 (Mahalanobis Distance check 省略...)
    
    state_ = state_ + K * y_tilde;
    P_ = (Eigen::Matrix2d::Identity() - K * H) * P_;
}