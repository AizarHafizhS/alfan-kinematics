#ifndef FOOT_TRAJECTORY_HPP
#define FOOT_TRAJECTORY_HPP

#include "alfan_walking/quintic_spline_1d.hpp"
#include <kdl/frames.hpp>

class FootTrajectory3D {
private:
    QuinticSpline1D spline_x_;
    QuinticSpline1D spline_y_;
    QuinticSpline1D spline_z_up_;   
    QuinticSpline1D spline_z_down_; 

    double t_start_;
    double t_mid_;
    double t_end_;
    
    // Variabel batas waktu baru untuk sumbu Z
    double z_start_time_;
    double z_end_time_;

public:
    FootTrajectory3D() : t_start_(0.0), t_mid_(0.0), t_end_(0.0), z_start_time_(0.0), z_end_time_(0.0) {}

    void setSwingTrajectory(double t_start, double t_end, 
                            double start_x, double target_x, 
                            double start_y, double target_y, 
                            double ground_z, double step_height) {
        t_start_ = t_start;
        t_end_   = t_end;
        double duration = t_end - t_start;
        
        // Sumbu X dan Y bergerak penuh sepanjang waktu fase (0% - 100%)
        spline_x_.computeCoefficients(t_start_, t_end_, start_x, target_x);
        spline_y_.computeCoefficients(t_start_, t_end_, start_y, target_y);

        // --- KUNCI PENYELESAIAN DRAGGING (PHASE MARGIN) ---
        // Kita paksa Sumbu Z menunggu 20% waktu berjalan agar CoM bergeser dulu,
        // dan memaksa Z mendarat 20% lebih awal sebelum CoM kembali.
        double margin = duration * 0.20; 
        
        z_start_time_ = t_start + margin;
        z_end_time_   = t_end - margin;
        t_mid_        = z_start_time_ + (z_end_time_ - z_start_time_) / 2.0;

        // Spline Z kini memiliki durasi yang lebih padat/singkat di udara
        spline_z_up_.computeCoefficients(z_start_time_, t_mid_, ground_z, ground_z + step_height);
        spline_z_down_.computeCoefficients(t_mid_, z_end_time_, ground_z + step_height, ground_z);
    }

    void setStanceTrajectory(double t_start, double t_end,
                             double start_x, double target_x,
                             double start_y, double target_y,
                             double ground_z) {
        t_start_ = t_start;
        t_end_   = t_end;
        z_start_time_ = t_end + 1.0; // Matikan pembacaan Z_up/down

        spline_x_.computeCoefficients(t_start_, t_end_, start_x, target_x);
        spline_y_.computeCoefficients(t_start_, t_end_, start_y, target_y);
        
        // Z dikunci statis di tanah
        spline_z_up_.computeCoefficients(t_start_, t_end_, ground_z, ground_z);
    }

    KDL::Vector getPosition(double t) const {
        double x = spline_x_.getPosition(t);
        double y = spline_y_.getPosition(t);
        double z = 0.0;

        // Routing Z dengan gerbang waktu margin yang baru
        if (t < z_start_time_ || t > z_end_time_) {
            z = 0.0; // Kaki dijamin rata dengan lantai di 20% awal dan akhir
        } else if (t <= t_mid_) {
            z = spline_z_up_.getPosition(t);
        } else {
            z = spline_z_down_.getPosition(t);
        }

        return KDL::Vector(x, y, z);
    }
};

#endif // FOOT_TRAJECTORY_HPP