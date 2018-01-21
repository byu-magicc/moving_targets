#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <memory>
#include <vector>
#include <utility>
#include <iostream>


namespace motion {

    typedef std::pair<double, double> coord_t;
    typedef std::vector<coord_t>      waypoints_t;
    typedef std::vector<coord_t>      path_t;

    class UnicyclePlanner
    {
    public:
        UnicyclePlanner();

        void updateState(double x, double y, double theta);

        void getCommands(double dt, double& v, double& w);

        void generateWaypoints(const waypoints_t& waypoints, const coord_t& vel);
        void generateCircle(double radius, const coord_t& center);
        void generateSinusoidal();
        void generateLemniscate(double a, const coord_t& center);


    private:
        // available types of trajectories
        enum Trajectory { WAYPOINTS, CIRCLE, SINUSOIDAL, LEMNISCATE };

        // unicycle state in the world frame
        double x_ = 0;
        double y_ = 0;
        double theta_ = 0;

        // trajectory
        Trajectory type_;
        path_t robotpath_;
        int traj_idx_ = 0;

        // parameters
        double d_ = 1;    // look-ahead distance for speed control

            // WAYPOINTS
            waypoints_t waypoints_;
            coord_t vel_;

            // CIRCLE
            double r_;
            coord_t center_;

            // LEMNISCATE
            double a_;
            // coord_t center_;

        // Motion controllers
        double speedControl(double dt, double ex, double ey);
        double headingControl(double dt, double ex, double ey);

        // Trajectory generation
        void regenerateTrajectory(double dt);
        double getCurrentGoalFromTrajectory(double& gx, double& gy);
        path_t mstraj(const waypoints_t& segments, const coord_t& qdmax, double dt);

        //
        // Simple PID class
        //

        class SimplePID
        {
        public:
            SimplePID(double kp, double ki, double kd, double min, double max, double tau=0.05)
                : kp_(kp), ki_(ki), kd_(kd), min_(min), max_(max), tau_(tau),
                  int_(0), diff_(0), ed1_(0) {};

            double compute(double e, double dt) {
                // update differentiator
                if (kd_ != 0) {
                double a1 = (2*tau_ - dt) / (2*tau_ + dt); double a2 = 2/(2*tau_ + dt);
                diff_ = a1*diff_ + a2*(e-ed1_); // Tustin discretization
                }
                // update integrator
                if (ki_ != 0) int_ += (dt/2)*(e-ed1_); // trapezoidal rule
                // implement PID control
                double u_unsat = kp_*e + ki_*int_ + kd_*diff_;
                // saturate command signal
                double u = u_unsat; if (u_unsat > max_) u = max_; if (u_unsat < min_) u = min_;
                // integrator anti-windup
                if (ki_ != 0) int_ += (dt/ki_)*(u-u_unsat);
                // reset if not finite
                if (!std::isfinite(u)) { diff_ = 0; int_ = 0; u = 0; }
                // return command signal
                return u;
            }

        private:
            double kp_, ki_, kd_; // gains
            double int_, diff_;   // integrator and differentiator
            double ed1_;          // error from last timestep
            double min_, max_;    // saturation limits
            double tau_;          // dirty-derivative bandwidth

        };

        std::unique_ptr<SimplePID> speedPID_;
        std::unique_ptr<SimplePID> headingPID_;
        
    };

    namespace {
        // Support pair addition
        template <typename T>
        std::pair<T,T> operator+(const std::pair<T,T>& l, const std::pair<T,T>& r) {
            return {l.first+r.first, l.second+r.second};
        }

        // Support pair differencing
        template <typename T>
        std::pair<T,T> operator-(const std::pair<T,T>& l, const std::pair<T,T>& r) {
            return {l.first-r.first, l.second-r.second};
        }

        // Support pair element-wise division
        template <typename T>
        std::pair<T,T> operator/(const std::pair<T,T>& l, const std::pair<T,T>& r) {
            return {l.first/r.first, l.second/r.second};
        }

        // Support pair and primitive division
        template <typename T>
        std::pair<T,T> operator/(const std::pair<T,T>& l, const T& r) {
            return {l.first/r, l.second/r};
        }

        // Support pair and primitive multiplication
        template <typename T>
        std::pair<T,T> operator*(const std::pair<T,T>& l, const T& r) {
            return {l.first*r, l.second*r};
        }
    }

}