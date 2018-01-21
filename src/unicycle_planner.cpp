#include "unicycle_planner.h"

namespace motion {

UnicyclePlanner::UnicyclePlanner() {
    speedPID_ = std::unique_ptr<SimplePID>(new SimplePID(6, 0.5, 0, -200, 200));
    headingPID_ = std::unique_ptr<SimplePID>(new SimplePID(6, 0, 0, -200, 200));
}

// ----------------------------------------------------------------------------

void UnicyclePlanner::updateState(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
}

// ----------------------------------------------------------------------------

void UnicyclePlanner::generateWaypoints(const waypoints_t& waypoints) {
    coord_t vel = {2, 2};
    robotpath_ = mstraj(waypoints, vel, 0.002);
}

// ----------------------------------------------------------------------------

void UnicyclePlanner::getCommands(double dt, double& v, double& w) {

    motion::waypoints_t waypoints = {{5,5},{10,8},{7,0},{15,-3},{15,4},{-2,4}};
    coord_t vel = {4, 4};
    robotpath_ = mstraj(waypoints, vel, dt);

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
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

double UnicyclePlanner::speedControl(double dt, double ex, double ey) {
    
    // calculate distance error using "look-ahead" distance. The purpose of
    // this is to help generate smoother commands
    double de = std::sqrt(ex*ex + ey*ey) - d_;

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

double UnicyclePlanner::getCurrentGoalFromTrajectory(double& gx, double& gy) {
    coord_t pos = robotpath_[traj_idx_++];
    gx = pos.first;
    gy = pos.second;

    std::cout << "[UnicyclePlanner] pos: (" << gx << ", " << gy << ")" << std::endl;
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

}