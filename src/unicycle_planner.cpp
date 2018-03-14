#include "unicycle_planner.h"

namespace motion {

UnicyclePlanner::UnicyclePlanner() {

}

// ----------------------------------------------------------------------------

void UnicyclePlanner::updateState(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
}

// ----------------------------------------------------------------------------

void UnicyclePlanner::goToPoint(double x, double y) {
    type_ = POINT;
    robotpath_.clear();
    robotpath_.push_back({x, y});

    d_ = 0;

    speedPID_ = std::unique_ptr<SimplePID>(new SimplePID(0.5, 0, 0, -max_vel_, max_vel_));
    headingPID_ = std::unique_ptr<SimplePID>(new SimplePID(4, 0, 0, -max_ang_vel_, max_ang_vel_));
}

// ----------------------------------------------------------------------------

void UnicyclePlanner::generateWaypoints(const waypoints_t& waypoints, const coord_t& vel) {
    type_ = WAYPOINTS;
    waypoints_ = waypoints;
    vel_ = vel;

    d_ = 0;

    max_vel_ = 5;
    max_ang_vel_ = (2*M_PI)/3.0;
    speedPID_ = std::unique_ptr<SimplePID>(new SimplePID(1, 0.1, 0, -max_vel_, max_vel_));
    headingPID_ = std::unique_ptr<SimplePID>(new SimplePID(15, 0, 0, -max_ang_vel_, max_ang_vel_));
}

// ----------------------------------------------------------------------------

void UnicyclePlanner::generateCircle(double radius, const coord_t& center) {
    type_ = CIRCLE;
    r_ = radius;
    center_ = center;

    d_ = 0;

    max_vel_ = 5;
    max_ang_vel_ = (2*M_PI)/3.0;
    speedPID_ = std::unique_ptr<SimplePID>(new SimplePID(1, 0.1, 0, -max_vel_, max_vel_));
    headingPID_ = std::unique_ptr<SimplePID>(new SimplePID(15, 0, 0, -max_ang_vel_, max_ang_vel_));

    robotpath_.clear();
    double dt = 0.0015;
    int Nsteps = std::ceil((2*M_PI)/dt);
    for (int i=0; i<Nsteps; i++) {
        double ang = i*dt;
        coord_t q = {cos(ang), sin(ang)};
        robotpath_.push_back(q*r_ + center_);
    }
}

// ----------------------------------------------------------------------------

void UnicyclePlanner::generateLemniscate(double a, const coord_t& center) {
    type_ = LEMNISCATE;
    a_ = a;
    center_ = center;

    d_ = 0;

    max_vel_ = 5;
    max_ang_vel_ = 4*M_PI;
    speedPID_ = std::unique_ptr<SimplePID>(new SimplePID(1, 0.1, 0, -max_vel_, max_vel_));
    headingPID_ = std::unique_ptr<SimplePID>(new SimplePID(15, 0, 0, -max_ang_vel_, max_ang_vel_));

    robotpath_.clear();
    double dt = 0.0015;
    int Nsteps = std::ceil((M_PI/2.0)/dt); // length(-pi/4:dt:pi/4)
    for (int i=0; i<Nsteps; i++) {
        double ang = i*dt - M_PI/4.0;

        coord_t q = {cos(ang), sin(ang)};

        double x = a_*cos(ang) * sqrt(2*cos(2*ang));
        double y = a_*sin(ang) * sqrt(2*cos(2*ang));

        robotpath_.push_back({x + center_.first, y + center_.second});
    }

    for (int i=Nsteps; i>0; i--) {
        double ang = i*dt - M_PI/4.0;

        coord_t q = {cos(ang), sin(ang)};

        double x = a_*cos(ang) * sqrt(2*cos(2*ang));
        double y = a_*sin(ang) * sqrt(2*cos(2*ang));

        robotpath_.push_back({-x + center_.first, -y + center_.second});
    }
}

// ----------------------------------------------------------------------------

void UnicyclePlanner::getCommands(double dt, double& v, double& w) {

    regenerateTrajectory(dt);

    // Select the current goal position from the current trajectory.
    // Think of this as moving the carrot.
    double gx, gy;
    getCurrentGoalFromTrajectory(gx, gy);

    // Calculate the goal distance error
    double ex = gx - x_;
    double ey = gy - y_;

    // Generate the unicycle commands to achieve the goal position
    v = speedControl(dt, ex, ey);
    w = headingControl(dt, ex, ey);

    // std::cout << "[getCommands] Velocity Command: " << v << std::endl;
}

// ----------------------------------------------------------------------------

// void UnicyclePlanner::getCommands(const math::Vector3 pos, double& v, double& w) {

//     // Update next waypoint if needed. 
//     pathManager(waypoints_, pos);


// }

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

double UnicyclePlanner::speedControl(double dt, double ex, double ey) {
    
    // calculate distance error using "look-ahead" distance. The purpose of
    // this is to help generate smoother commands
    double de = std::sqrt(ex*ex + ey*ey) - d_;

    // std::cout << "[SpeedControl] Error: " << de << std::endl;

    // generate a speed command to regulate error to zero
    return speedPID_->compute(de, dt);
}

// ----------------------------------------------------------------------------

double UnicyclePlanner::headingControl(double dt, double ex, double ey) {
    
    // calculate heading to the goal
    double gamma = std::atan2(ey, ex);

    // calculate heading error
    double he = gamma - theta_;

    // angle wrap to keep he \in [-pi, pi]
    while (he >  M_PI) he -= 2*M_PI;
    while (he < -M_PI) he += 2*M_PI;

    // generate a heading rate command to regulate error to zero
    return headingPID_->compute(he, dt);
}

// ----------------------------------------------------------------------------

void UnicyclePlanner::regenerateTrajectory(double dt) {

    if (type_ == WAYPOINTS) {
        robotpath_ = mstraj(waypoints_, vel_, dt);
    }

}

// ----------------------------------------------------------------------------

double UnicyclePlanner::getCurrentGoalFromTrajectory(double& gx, double& gy) {

    coord_t pos;
    if (traj_idx_ >= robotpath_.size()) {
        pos = robotpath_[robotpath_.size()-1];
        traj_idx_ = 0;
    } else {
        pos = robotpath_[traj_idx_++];
    }

    
    gx = pos.first;
    gy = pos.second;

    // std::cout << "[UnicyclePlanner] pos: (" << gx << ", " << gy << ")" << std::endl;
}

// ----------------------------------------------------------------------------

path_t UnicyclePlanner::mstraj(const waypoints_t& segments, const coord_t& qdmax, double dt) {

    // num of segments
    int ns = segments.size();

    // initial conditions
    coord_t q_prev = segments[0];

    // keep track of points in the trajectory
    path_t traj;

    // loop through each segment endpoint, skipping the first
    for (int i=1; i<ns; i++) {
        // setup next target waypoint
        coord_t q_next = segments[i];       // current target
        coord_t q_diff = q_next - q_prev;   // total distance to move this segment

        //
        // qdmax is specified, so compute slowest dimension
        //

        // how long will it take to travel this segment?
        coord_t tl = q_diff / qdmax;
        // ensure divisibility by sample rate
        tl = {std::abs(tl.first), std::abs(tl.second)}; tl = tl / dt;
        tl = {std::ceil(tl.first), std::ceil(tl.second)}; tl = tl * dt;

        // find the total time the slowest dimension takes
        double tseg = (tl.first > tl.second) ? tl.first : tl.second;

        //
        // create the trajectory for this segment
        //

        // linear velocity from q_prev to q_next
        coord_t qd = q_diff / tseg;

        // how many dt steps are in tseg?
        int Nsteps = tseg / dt;

        for (int j=0; j<Nsteps; j++) {
            double s = (j*dt)/tseg;
            coord_t q = q_prev*(1-s) + q_next*s; // linear step
            traj.push_back(q);
        }

        // the current target (q_next) is now behind us.
        q_prev.swap(q_next);
    }

    return traj;

}

// void UnicyclePlanner::pathManager(const waypoints_t& segments, const math::Vector3 pos) {

//     float radius = 1;

//     // Distance between current position and desired waypoint
//     float distance = std::sqrt(pow(segments[1].first - pos.x,2) + pow(segments[1].second-pos.y,2));

//     // switch to next waypoint. 
//     if (distance < radius) {

//         segments.push_back(segments[0]);
//         segments.erase(segments.begin());
//     }


// }

}