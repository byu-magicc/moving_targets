#pragma once

#define _USE_MATH_DEFINES

#include "waypoint.h"
#include "math.h"



namespace motion {

	class LinearPath {

	public:

		LinearPath() = default;
		
		void followStrightLine(coord_t waypoint, coord_t pos, double heading);

	
	};



}