#include "path_manager/radius_manager.h"

namespace motion {
   
    bool RadiusManager::manage_waypoints(const coord_t& pos, waypoints_t& waypoints, float& distance) {

        // Distance between current position and desired waypoint
        coord_t vect = waypoints[1]-pos;
        distance = get_magnitude(vect);

        // check if we need to switch to next waypoint
        bool wp_reached = (distance < radius_);

        if (wp_reached && should_cycle_) {
            // cycle waypoints (length of vector is constant)
            waypoints.push_back(waypoints[0]);
            waypoints.erase(waypoints.begin());
        }

        return wp_reached;
    }
}