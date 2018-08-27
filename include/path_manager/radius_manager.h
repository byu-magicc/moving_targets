#pragma once

#include "waypoint.h"
#include "path_manager/base_manager.h"

namespace motion {

    class RadiusManager : public BaseManager
    {
    public:
        RadiusManager() = default;

        bool manage_waypoints(const coord_t& pos, waypoints_t& waypoints, float& distance) override;

    private:
        // If the agent is within this distance to the waypoint
        // Then the agent will start moving to the next waypoint
        float radius_ = 1;
    };


}