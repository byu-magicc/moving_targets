/**
 * @file random_waypoints.h
 * @author Parker Lusk <parkerclusk@gmail.com>
 */

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <memory>

#include "waypoint.h"
#include "path_follower/base_follower.h"
#include "path_follower/straight_line.h"

namespace motion {

    class RandomWaypoints : public BaseFollower {

    public:

        RandomWaypoints(const waypoints_t& bbox);
       
        /**
         * @brief      Generate commands for random waypoints
         *
         * @param[in]  p     current position of agent
         * @param[in]  chi   current heading of agent
         *
         * @return     desired altitude and heading
         */
        FollowerCommands randomize(waypoints_t& waypoints, bool wp_reached, const coord_t& p, const double& chi) override;

        void set_parameters(const FollowerParams& params) override;

        /**
         * @brief      Initialize the agent's initial state within constrained region
         *
         * @return     initial position
         */
        coord_t initialize();

    private:
        /**
         * internal straight line follower used to follow random waypoints
         */
        std::unique_ptr<StraightLine> straight_line_;

        coord_t waypoint_prev_;         ///< single current waypoint
        coord_t waypoint_curr_;         ///< single next waypoint
        coord_t waypoint_next_;         ///< single next+1 waypoint (three waypoints required for half-plane)

        double min_x_, min_y_, min_z_;  ///< minimum parameter of uniform distribution
        double max_x_, max_y_, max_z_;  ///< maximum parameter of uniform distribution

        /**
         * @brief      Sample a waypoint from 3 indep uniform distributions
         *
         * @return     a random waypoint in the configuration region
         */
        coord_t sample_waypoint();
    };
}