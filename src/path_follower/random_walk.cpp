/**
 * @file random_walk.h
 * @author Parker Lusk <parkerclusk@gmail.com>
 */

#include "path_follower/random_walk.h"

#include <random>

namespace motion {

RandomWalk::RandomWalk(const coord_t& origin, const waypoints_t& bbox) {
    // initialize straight line follower
    straight_line_ = std::unique_ptr<motion::StraightLine>(new motion::StraightLine());

    // there should be two waypoints inside bbox for bounding box specs:
    //    [0] == bottom-left
    //    [1] == top-right
    min_x_ = std::get<0>(bbox[0]);
    min_y_ = std::get<1>(bbox[0]);
    min_z_ = std::get<2>(bbox[0]);

    max_x_ = std::get<0>(bbox[1]);
    max_y_ = std::get<1>(bbox[1]);
    max_z_ = std::get<2>(bbox[1]);

    // origin of bounding box -- used to shift sampled waypoints
    origin_ = origin;
}

//----------------------------------------------------------------------

FollowerCommands RandomWalk::random_walk(const coord_t& p, const double& chi) {

    // calculate distance error
    auto wp_diff = waypoint_next_-waypoint_current_;

    // generate commands using a standard straight line waypoint follower
    auto commands = straight_line_->line_follower(waypoint_current_, wp_diff, p, chi);

    // check if we should step waypoints for next time
    // (i.e., are we there yet?)
    auto pos_error = waypoint_next_ - p;
    if (get_magnitude(pos_error) < rho_) {
        waypoint_current_ = waypoint_next_;
        waypoint_next_ = sample_waypoint();

        std::cout << std::endl;
        std::cout << "WP Next: " << waypoint_next_ << std::endl;
        std::cout << std::endl;
    }

    return commands;
}

//----------------------------------------------------------------------

void RandomWalk::set_parameters(const FollowerParams& params) {
    rho_ = params.rho;
    straight_line_->set_parameters(params);
}

//----------------------------------------------------------------------

coord_t RandomWalk::initialize() {

    // sample two waypoints
    waypoint_current_ = sample_waypoint();
    waypoint_next_ = sample_waypoint();

    std::cout << std::endl;
    std::cout << "WP Current: " << waypoint_current_ << std::endl;
    std::cout << "WP Next: " << waypoint_next_ << std::endl;
    std::cout << std::endl;

    return waypoint_current_;
}

//----------------------------------------------------------------------
// Private Methods
//----------------------------------------------------------------------

coord_t RandomWalk::sample_waypoint() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> X(min_x_, max_x_);
    std::uniform_real_distribution<> Y(min_y_, max_y_);
    std::uniform_real_distribution<> Z(min_z_, max_z_);

    // shifted random waypoint based on origin
    return (coord_t(X(gen), Y(gen), Z(gen)) + origin_);
}

}
