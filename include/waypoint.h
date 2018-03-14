#pragma once

namespace waypoint {

	typedef std::tuple<double, double, double> coord_t;
	typedef std::vector<coord_t>      waypoints_t;

	// Support tuple addition
	template <typename T>
	std::tuple<T,T,T> operator+(const std::tuple<T,T,T>& l, const std::tuple<T,T,T>& r) {
	    return {std::get<0>(l)+std::get<0>(r), std::get<1>(l)+std::get<1>(r),std::get<2>(l)+std::get<2>(r)};
	}

	// Support tuple differencing
	template <typename T>
	std::tuple<T,T,T> operator-(const std::tuple<T,T,T>& l, const std::tuple<T,T,T>& r) {
	    return {std::get<0>(l)-std::get<0>(r), std::get<1>(l)-std::get<1>(r),std::get<2>(l)-std::get<2>(r)};
	}

	// Support tuple element-wise division
	template <typename T>
	std::tuple<T,T,T> operator/(const std::tuple<T,T,T>& l, const std::tuple<T,T,T>& r) {
	    return {std::get<0>(l)/std::get<0>(r), std::get<1>(l)/std::get<1>(r),std::get<2>(l)/std::get<2>(r)};
	}

	// Support tuple and primitive division
	template <typename T>
	std::tuple<T,T,T> operator/(const std::tuple<T,T,T>& l, const T& r) {
	    return {std::get<0>(l)/r, std::get<1>(l)/r,std::get<2>(l)/r};
	}

	// Support tuple element-wise multiplication
	template <typename T>
	std::tuple<T,T,T> operator/(const std::tuple<T,T,T>& l, const std::tuple<T,T,T>& r) {
	    return {std::get<0>(l)*std::get<0>(r), std::get<1>(l)*std::get<1>(r),std::get<2>(l)*std::get<2>(r)};
	}

	// Support tuple and primitive multiplication
	template <typename T>
	std::tuple<T,T,T> operator*(const std::tuple<T,T,T>& l, const T& r) {
	    return {std::get<0>(l)*r, std::get<1>(l)*r,std::get<2>(l)*r};
	}





}
