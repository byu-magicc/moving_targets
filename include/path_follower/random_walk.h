/**
 * @file random_walk.h
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

    class RandomWalk : public BaseFollower {

    public:

        RandomWalk(const coord_t& origin, const waypoints_t& bbox);
       
        /**
         * @brief      Generate commands for random walk
         *
         * @param[in]  p     current position of agent
         * @param[in]  chi   current heading of agent
         *
         * @return     desired altitude and heading
         */
        FollowerCommands random_walk(waypoints_t& waypoints, bool wp_reached, const coord_t& p, const double& chi) override;

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

        coord_t waypoint_prev_;      ///< single current waypoint
        coord_t waypoint_curr_;         ///< single next waypoint
        coord_t waypoint_next_;    ///< single next+1 waypoint (three waypoints required for half-plane)

        double min_x_, min_y_, min_z_;  ///< minimum parameter of uniform distribution
        double max_x_, max_y_, max_z_;  ///< maximum parameter of uniform distribution

        coord_t origin_;                ///< origin of bounding box

        coord_t sample_waypoint();
    };
}