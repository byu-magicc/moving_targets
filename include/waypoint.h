#pragma once

#include <tuple>
#include <vector>
#include <iostream>
#include <math.h> 

namespace motion {

    typedef std::tuple<double, double, double>  coord_t;
    typedef std::vector<coord_t>                waypoints_t;

    // calculate the magnitude of a waypoint
    inline double get_magnitude(coord_t l) {
        return sqrt(pow(std::get<0>(l),2) + pow(std::get<1>(l),2)+pow(std::get<2>(l),2));
    }
}

// Support tuple addition
template <typename T>
std::tuple<T,T,T> operator+(const std::tuple<T,T,T>& l, const std::tuple<T,T,T>& r) {
    return std::make_tuple(std::get<0>(l)+std::get<0>(r), std::get<1>(l)+std::get<1>(r),std::get<2>(l)+std::get<2>(r));
}

// Support tuple differencing
template <typename T>
std::tuple<T,T,T> operator-(const std::tuple<T,T,T>& l, const std::tuple<T,T,T>& r) {
    return std::make_tuple(std::get<0>(l)-std::get<0>(r), std::get<1>(l)-std::get<1>(r),std::get<2>(l)-std::get<2>(r));
}

// Support tuple element-wise division
template <typename T>
std::tuple<T,T,T> operator/(const std::tuple<T,T,T>& l, const std::tuple<T,T,T>& r) {
    return std::make_tuple(std::get<0>(l)/std::get<0>(r), std::get<1>(l)/std::get<1>(r),std::get<2>(l)/std::get<2>(r));
}

// Support tuple and primitive division
template <typename T, typename P>
std::tuple<T,T,T> operator/(const std::tuple<T,T,T>& l, const P& r) {
    return std::make_tuple(std::get<0>(l)/r, std::get<1>(l)/r,std::get<2>(l)/r);
}

// Support tuple element-wise multiplication
template <typename T>
std::tuple<T,T,T> operator*(const std::tuple<T,T,T>& l, const std::tuple<T,T,T>& r) {
    return std::make_tuple(std::get<0>(l)*std::get<0>(r), std::get<1>(l)*std::get<1>(r),std::get<2>(l)*std::get<2>(r));
}

// Support tuple and primitive multiplication
template <typename T, typename P>
std::tuple<T,T,T> operator*(const std::tuple<T,T,T>& l, const P& r) {
    return std::make_tuple(std::get<0>(l)*r, std::get<1>(l)*r,std::get<2>(l)*r);
}


// For printing contents
template<std::size_t> struct int_{};

template<typename T, size_t Pos>
std::ostream& print_tuple(std::ostream& out, const T& t, int_<Pos>) {
    out << std::get< std::tuple_size<T>::value-Pos >(t) << ',';
     return print_tuple(out, t, int_<Pos-1>());
}

template <typename T>
std::ostream& print_tuple(std::ostream& out, const T& t, int_<1> ) {
  return out << std::get<std::tuple_size<T>::value-1>(t);
}

template <class... Args>
std::ostream& operator<<(std::ostream& out, const std::tuple<Args...>& t) {
  out << '('; 
  print_tuple(out, t, int_<sizeof...(Args)>()); 
  return out << ')';
}