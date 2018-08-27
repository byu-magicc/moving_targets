/**
 * @file half_plane_manager.h
 * @author Parker Lusk <parkerclusk@gmail.com>
 */

#pragma once

#include "waypoint.h"
#include "path_manager/base_manager.h"

namespace motion {

    class HalfPlaneManager : public BaseManager
    {
    public:
        HalfPlaneManager() = default;

        bool manage_waypoints(const coord_t& pos, waypoints_t& waypoints, float& distance) override;

    private:
        /**
         * @brief      Check if point p is in the halfplane defined by r and n (Algorithm 5, UAVBook)
         *
         * @param[in]  p     the point we are asking about being in the half plane
         * @param[in]  r     a point on the plane where the normal is coming from (i.e., the current waypoint)
         * @param[in]  n     the normal to the plane, used to separate two waypoints
         *
         * @return     whether or not $p \in \mathcal{H}(r, n)$
         */
        bool in_halfplane(const coord_t& p, const coord_t& r, const coord_t& n);

        static constexpr double TOO_FAR = 15;      ///< error threshold for counting current wp as reached
    };


}