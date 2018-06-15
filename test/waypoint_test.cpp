#include "waypoint.h"
#include <iostream>

int main() {

	waypoint::coord_t point1 (1,1,1);
	waypoint::coord_t point2 (2,2,2);

	waypoint::coord_t test_add;
	waypoint::coord_t test_subtract;
	waypoint::coord_t test_elem_div;
	waypoint::coord_t test_div;
	waypoint::coord_t test_elem_mult;
	waypoint::coord_t test_mult;

	std::cout << "Point1: " << point1 << std::endl;
	std::cout << "Point2: " << point2 << std::endl;

	test_add = point1 + point2;
	std::cout << "Point1 + Point2: " << test_add << std::endl;

	test_subtract = point1 - point2;
	std::cout << "Point1 - Point2: " << test_subtract << std::endl;

	test_elem_div = point1/point2;
	std::cout << "Point1 / Point2: " << test_elem_div << std::endl;

	test_div = point1/4;
	std::cout << "Point1 / 4: " << test_div << std::endl;

	test_elem_mult = point1*point2;
	std::cout << "Point1 * Point2: " << test_elem_mult << std::endl;

	test_mult = point1*4.0;
	std::cout << "Point1 * 4: " << test_div << std::endl;




	

	return 0;

}