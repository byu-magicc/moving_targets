/**
 * @file base_manager.h
 * @author Parker Lusk <parkerclusk@gmail.com>
 */

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include "waypoint.h"

namespace motion {

    class BaseManager
    {
    public:
        /**
         * @brief      selects current waypoint from list
         *
         * @param[in]  pos        current agent position
         * @param      waypoints  remaining waypoints to reach
         * @param[in]  traj       the type of trajectory
         * @param      distance   remaining distance to reach current waypoint
         *
         * @return     Boolean indicating if current waypoint was reached
         */
        virtual bool manage_waypoints(const coord_t& pos, waypoints_t& waypoints, float& distance) = 0;

        virtual void set_cycling(bool should_cycle) { should_cycle_ = should_cycle; }
    protected:
        bool should_cycle_;         ///< whether or not waypoints should be cycled when the current waypoint is reached
    };
}
