#ifndef MATH_SIMPLE_HPP
#define MATH_SIMPLE_HPP


namespace ros_math_simple {
    inline double clamp(double val, double min, double max)
    {
        const double t = val < min ? min: val;
        return t > max ? max : t;
    }
}

#endif