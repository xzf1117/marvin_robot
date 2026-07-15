#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

class SE3ReferenceGenerator {
public:
    SE3ReferenceGenerator(double pose_cutoff,
                          double vel_cutoff)
        : pose_cutoff_(pose_cutoff),
          vel_cutoff_(vel_cutoff),
          first_time_(true)
    {
        v_filtered_.setZero();
    }

    // ================================
    // 70Hz measurement update
    // ================================
    void updateMeasurement(const Eigen::Isometry3d& T_meas,
                           double timestamp)
    {
        if (first_time_) {
            T_filtered_ = T_meas;
            last_meas_time_ = timestamp;
            first_time_ = false;
            return;
        }

        double dt = timestamp - last_meas_time_;
        if (dt <= 0.0)
            return;

        last_meas_time_ = timestamp;

        // 1️⃣ pose error
        Eigen::Isometry3d Delta =
            T_filtered_.inverse() * T_meas;

        Eigen::Matrix<double,6,1> xi =
            se3Log(Delta);

        // 2️⃣ twist estimation
        Eigen::Matrix<double,6,1> v_meas =
            xi / dt;

        // 3️⃣ low-pass twist
        double alpha_v = computeAlpha(vel_cutoff_, dt);
        v_filtered_ =
            alpha_v * v_meas +
            (1.0 - alpha_v) * v_filtered_;

        // 4️⃣ low-pass pose correction
        double alpha_p = computeAlpha(pose_cutoff_, dt);
        Eigen::Matrix<double,6,1> xi_filtered =
            alpha_p * xi;

        T_filtered_ =
            T_filtered_ * se3Exp(xi_filtered);
    }

    // ================================
    // 200Hz control query
    // ================================
    Eigen::Isometry3d getReference(double current_time)
    {
        if (first_time_)
            return T_filtered_;

        double dt =
            current_time - last_meas_time_;

        if (dt < 0.0)
            dt = 0.0;

        // 恒速度外推
        return T_filtered_ *
               se3Exp(v_filtered_ * dt);
    }

private:

    double computeAlpha(double cutoff,
                        double dt) const
    {
        double tau =
            1.0 / (2.0 * M_PI * cutoff);
        return dt / (tau + dt);
    }

    // ===== Lie algebra utilities =====

    Eigen::Matrix3d skew(
        const Eigen::Vector3d& v) const
    {
        Eigen::Matrix3d m;
        m << 0, -v.z(), v.y(),
             v.z(), 0, -v.x(),
             -v.y(), v.x(), 0;
        return m;
    }

    Eigen::Vector3d so3Log(
        const Eigen::Matrix3d& R) const
    {
        Eigen::AngleAxisd aa(R);
        return aa.axis() * aa.angle();
    }

    Eigen::Matrix3d so3Exp(
        const Eigen::Vector3d& w) const
    {
        double theta = w.norm();
        if (theta < 1e-8)
            return Eigen::Matrix3d::Identity();

        return Eigen::AngleAxisd(theta,
                                 w.normalized())
            .toRotationMatrix();
    }

    Eigen::Matrix<double,6,1>
    se3Log(const Eigen::Isometry3d& T) const
    {
        Eigen::Matrix<double,6,1> xi;

        Eigen::Matrix3d R = T.rotation();
        Eigen::Vector3d t = T.translation();

        Eigen::Vector3d omega = so3Log(R);
        double theta = omega.norm();

        Eigen::Matrix3d V_inv =
            Eigen::Matrix3d::Identity();

        if (theta > 1e-8) {
            Eigen::Matrix3d omega_hat =
                skew(omega / theta);

            double half_theta = 0.5 * theta;

            V_inv -= 0.5 * omega_hat;
            V_inv +=
                (1.0/theta -
                 0.5/std::tan(half_theta))
                * omega_hat * omega_hat;
        }

        Eigen::Vector3d rho = V_inv * t;

        xi.head<3>() = rho;
        xi.tail<3>() = omega;

        return xi;
    }

    Eigen::Isometry3d
    se3Exp(const Eigen::Matrix<double,6,1>& xi) const
    {
        Eigen::Vector3d rho = xi.head<3>();
        Eigen::Vector3d omega = xi.tail<3>();

        double theta = omega.norm();

        Eigen::Matrix3d R = so3Exp(omega);
        Eigen::Matrix3d V =
            Eigen::Matrix3d::Identity();

        if (theta > 1e-8) {
            Eigen::Matrix3d omega_hat =
                skew(omega / theta);

            V += (1 - std::cos(theta)) / theta
                 * omega_hat;
            V += (theta - std::sin(theta)) / theta
                 * omega_hat * omega_hat;
        }

        Eigen::Vector3d t = V * rho;

        Eigen::Isometry3d T =
            Eigen::Isometry3d::Identity();

        T.linear() = R;
        T.translation() = t;

        return T;
    }

private:
    double pose_cutoff_;
    double vel_cutoff_;

    bool first_time_;
    double last_meas_time_;

    Eigen::Isometry3d T_filtered_;
    Eigen::Matrix<double,6,1> v_filtered_;
};