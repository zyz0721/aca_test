#pragma once
#include "mpc_common.h"

class ReferenceGenerator {
public:
    void init(const MpcParams &p, int nx, int nu) { 
        p_ = p;
        nx_ = p.solver_cfg.nx;
        nu_ = p.solver_cfg.nu; 
    }
    double normalizeAngle(double raw_yaw) const{
        if(std::isnan(raw_yaw)){
            return 0.0;
        }
        raw_yaw = std::fmod(raw_yaw, 2.0 * M_PI);
        if (raw_yaw <= -M_PI) raw_yaw += 2.0 * M_PI;
        if (raw_yaw > M_PI) raw_yaw -= 2.0 * M_PI;
        return raw_yaw; 
    }
    RefState at(double t) const {
        RefState r(nx_, nu_);

        if (p_.ref_type == "circle") {
            double omega = p_.circle_v / p_.circle_r;
            double phi = omega * t;
            // 位置
            r.val[0] = p_.circle_r * sin(phi);
            r.val[1] = p_.circle_r * (1.0 - cos(phi));
            // r.val[2] = phi;   // yaw = 切线方向
            r.val[2] = normalizeAngle(phi);
            // 全局速度
            r.val[3] = p_.circle_v * cos(phi);  // vx_global
            r.val[4] = p_.circle_v * sin(phi);  // vy_global
            r.val[5] = omega;                   // dyaw
            // 稳态加速度 (向心加速度)
            r.u_ref[0] = -p_.circle_v * omega * sin(phi);  // ax
            r.u_ref[1] =  p_.circle_v * omega * cos(phi);  // ay
            r.u_ref[2] = 0.0;                              // ddyaw
        }
        else if (p_.ref_type == "line") {
            r.val[0] = p_.line_vx * t;
            r.val[1] = 0.0;
            r.val[2] = 0.0;
            r.val[3] = p_.line_vx;  // vx
            r.val[4] = 0.0;         // vy
            r.val[5] = 0.0;         // dyaw
            r.u_ref[0] = 0.0;
            r.u_ref[1] = 0.0;
            r.u_ref[2] = 0.0;
        }
        else if (p_.ref_type == "figure8") {
            double w = p_.fig8_omega;
            double a = p_.fig8_a;
            r.val[0] = a * sin(w * t);
            r.val[1] = a * sin(2.0 * w * t) / 2.0;
            // 全局速度 (解析导数)
            double vx = a * w * cos(w * t);
            double vy = a * w * cos(2.0 * w * t);
            r.val[2] = atan2(vy, vx);  // yaw = 切线方向
            r.val[3] = vx;
            r.val[4] = vy;
            r.val[5] = 0.0;  // 简化: dyaw=0
            // 加速度 (解析二阶导)
            r.u_ref[0] = -a * w * w * sin(w * t);
            r.u_ref[1] = -2.0 * a * w * w * sin(2.0 * w * t);
            r.u_ref[2] = 0.0;
        }
        return r;
    }

    std::vector<RefState> full_trajectory(int n_pts) const {
        std::vector<RefState> pts(n_pts + 1);
        double T_full = period();
        for (int i = 0; i <= n_pts; i++) {
            pts[i] = at(T_full * i / n_pts);
        }
        return pts;
    }

    double period() const {
        if (p_.ref_type == "circle")
            return 2.0 * M_PI * p_.circle_r / p_.circle_v;
        else if (p_.ref_type == "figure8")
            return 2.0 * M_PI / p_.fig8_omega;
        else
            return 10.0;
    }

private:
    MpcParams p_;
    int nx_ = 0, nu_ = 0;
};