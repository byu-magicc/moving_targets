#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include "waypoint.h"

namespace motion {

    // The two commands
    struct FollowerCommands {

        float chi_c;   // Heading commands
        float h_c;     // Altitude commands

    };

    struct FollowerParams {

        // Orbit follower parameters
        float k_orbit;   // Gain
        float rho;       // Orbit radius
        float lambda;    // Direction of orbit

        // Straight Line follower parameters
        float k_path;        // Gain
        float chi_infinity;  // Gain

        // trajectory
        int traj;

    };

    class BaseFollower {

    public:

        virtual FollowerCommands line_follower(const coord_t& r, const coord_t& q, const coord_t& p, const double& chi) {}

        virtual FollowerCommands orbit_follower(const coord_t& c, const coord_t& p, const double chi) {}

        virtual FollowerCommands randomize(waypoints_t& waypoints, bool wp_reached, const coord_t& p, const double& chi) {}

        virtual void set_parameters(const FollowerParams& params) = 0;
    };
}
