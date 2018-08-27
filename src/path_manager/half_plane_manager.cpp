/**
 * @file half_plane_manager.cpp
 * @author Parker Lusk <parkerclusk@gmail.com>
 */

#include "path_manager/half_plane_manager.h"

namespace motion {
    
bool HalfPlaneManager::manage_waypoints(const coord_t& pos, waypoints_t& waypoints, float& distance) {
    if (waypoints.size() < 3) {
        std::cout << "[HalfPlaneManager] There must be at least 3 waypoints" << std::endl;
        distance = 0;
        return false;
    }

    // for convenience
    auto wp_prev = waypoints[0];
    auto wp_curr = waypoints[1];
    auto wp_next = waypoints[2];

    coord_t qm1 = (wp_curr-wp_prev)/get_magnitude(wp_curr-wp_prev); // Line 5, Algorithm 5 (p 191) of UAVBook
    coord_t q = (wp_next-wp_curr)/get_magnitude(wp_next-wp_curr);   // Line 6, Algorithm 5 (p 191) of UAVBook
    coord_t n = (qm1+q)/get_magnitude(qm1+q);                       // Line 7, Algorithm 5 (p 191) of UAVBook

    // check if we need to switch to next waypoint
    bool wp_reached = in_halfplane(pos, wp_curr, n);   

    if (wp_reached && should_cycle_) {
        // cycle waypoints (length of vector is constant)
        waypoints.push_back(waypoints[0]);
        waypoints.erase(waypoints.begin());
    }

    return wp_reached;
}

// ----------------------------------------------------------------------------

bool HalfPlaneManager::in_halfplane(const coord_t& p, const coord_t& r, const coord_t& n) {
    auto a = p - r;

    // inner product (p-r).'n
    auto ip = std::get<0>(a)*std::get<0>(n) + std::get<1>(a)*std::get<1>(n) + std::get<2>(a)*std::get<2>(n);

    return (ip >= 0);
}

} // namespace motion