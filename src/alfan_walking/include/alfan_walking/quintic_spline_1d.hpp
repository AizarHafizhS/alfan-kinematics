#ifndef QUINTIC_SPLINE_1D_HPP
#define QUINTIC_SPLINE_1D_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>

class QuinticSpline1D {
private:
    Eigen::VectorXd coefficients_; // Menyimpan c0, c1, c2, c3, c4, c5
    double t_start_;
    double t_end_;
    bool is_calculated_;

public:
    QuinticSpline1D() : coefficients_(6), t_start_(0.0), t_end_(0.0), is_calculated_(false) {
        coefficients_.setZero();
    }

    // Fungsi Kalkulator Matriks
    // Menghitung koefisien berdasarkan posisi, kecepatan, dan percepatan awal/akhir
    void computeCoefficients(double t_start, double t_end, 
                             double p0, double pf, 
                             double v0 = 0.0, double vf = 0.0, 
                             double a0 = 0.0, double af = 0.0) {
        t_start_ = t_start;
        t_end_ = t_end;
        double T = t_end - t_start;

        // Cegah pembagian dengan nol atau waktu terbalik
        if (T <= 0.0) {
            std::cerr << "[QuinticSpline] Error: Durasi waktu tidak valid (T <= 0)!" << std::endl;
            return;
        }

        // Trik Presisi Engineering:
        // Kita menggeser waktu awal (t_start) menjadi 0.0 saat kalkulasi matriks.
        // Jika kita memasukkan stempel waktu ROS 2 (misal: 1773251861.97) ke pangkat 5, 
        // variabel 'double' C++ akan mengalami overflow dan matriks akan gagal diselesaikan (Singular).
        Eigen::MatrixXd A(6, 6);
        A << 1, 0,   0,     0,      0,       0,          // Posisi saat t=0
             1, T,   pow(T,2), pow(T,3), pow(T,4), pow(T,5), // Posisi saat t=T
             0, 1,   0,     0,      0,       0,          // Kecepatan saat t=0
             0, 1,   2*T,   3*pow(T,2), 4*pow(T,3), 5*pow(T,4), // Kecepatan saat t=T
             0, 0,   2,     0,      0,       0,          // Percepatan saat t=0
             0, 0,   2,     6*T,    12*pow(T,2), 20*pow(T,3); // Percepatan saat t=T

        Eigen::VectorXd B(6);
        B << p0, pf, v0, vf, a0, af;

        // Menyelesaikan X = A^-1 * B menggunakan dekomposisi Householder QR (Paling stabil untuk matriks kecil)
        coefficients_ = A.colPivHouseholderQr().solve(B);
        is_calculated_ = true;
    }

    // Fungsi Pengambil Nilai Posisi
    double getPosition(double t) const {
        if (!is_calculated_) return 0.0;
        
        // Membatasi t agar tidak melewati batas (Clamping)
        if (t < t_start_) t = t_start_;
        if (t > t_end_) t = t_end_;

        double dt = t - t_start_; // Geser kembali t relatif terhadap t_start

        return coefficients_(0) + 
               coefficients_(1) * dt + 
               coefficients_(2) * pow(dt, 2) + 
               coefficients_(3) * pow(dt, 3) + 
               coefficients_(4) * pow(dt, 4) + 
               coefficients_(5) * pow(dt, 5);
    }
};

#endif // QUINTIC_SPLINE_1D_HPP