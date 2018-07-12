/*
 * vector3.hpp
 * 
 * Header file for an ROS based general-purpose Catmull-Rom Spline
 * 
 * Hajdu Csaba (kyberszittya)
 */
#ifndef CATMULL_ROS_VECTOR3_HPP
#define CATMULL_ROS_VECTOR3_HPP

namespace catmull_ros {

/**
 * catmull_ros::Vector3
 * 
 * Simple-purpose 3D vector suitable for all platforms (even for uCs)
 * Some polishing might be required though.
 * 
 * Hajdu Csaba (kyberszittya)
 */
struct Vector3
{
	double coords[3];
	
	Vector3() : coords{0.0, 0.0, 0.0} {}
	Vector3(double x, double y, double z) : coords{x,y,z} {}
	Vector3(const Vector3& v): coords{v.coords[0], v.coords[1], v.coords[2]}
	{
		
	}

	inline const double X()
	{
		return coords[0];
	}

	inline const double Y()
	{
		return coords[1];
	}

	inline const double Z()
	{
		return coords[2];
	}

	Vector3 operator+=(const Vector3& rhs)
	{
		coords[0] += rhs.coords[0];
		coords[1] += rhs.coords[1];
		coords[2] += rhs.coords[2];
		return *this;
	}
	
	Vector3 operator-=(const Vector3& rhs)
	{
		coords[0] -= rhs.coords[0];
		coords[1] -= rhs.coords[1];
		coords[2] -= rhs.coords[2];
		return *this;
	}

	Vector3 operator*=(const Vector3& rhs)
	{
		coords[0] *= rhs.coords[0];
		coords[1] *= rhs.coords[1];
		coords[2] *= rhs.coords[2];
		return *this;
	}

	Vector3 operator+=(const double& rhs)
	{
		coords[0] += rhs;
		coords[1] += rhs;
		coords[2] += rhs;
		return *this;
	}

	Vector3 operator/=(const double& rhs)
	{
		coords[0] /= rhs;
		coords[1] /= rhs;
		coords[2] /= rhs;
		return *this;
	}

	Vector3 operator*=(const double& rhs)
	{
		coords[0] *= rhs;
		coords[1] *= rhs;
		coords[2] *= rhs;
		return *this;
	}

	double GetSquaredNorm()
	{
		return coords[0]*coords[0]
			+coords[1]*coords[1]
			+coords[2]*coords[2];
	}

	double GetNorm()
	{
		return sqrt(coords[0]*coords[0]
			+coords[1]*coords[1]
			+coords[2]*coords[2]
		);
	}

};

inline Vector3 operator+(const Vector3& lhs, const Vector3& rhs)
{
	Vector3 res(lhs);
	res += rhs;
	return res;
}

inline Vector3 operator-(const Vector3& lhs, const Vector3& rhs)
{
	Vector3 res(lhs);
	res -= rhs;
	return res;
}

inline Vector3 operator*(const Vector3& lhs, const double& rhs)
{
	Vector3 res(lhs);
	res *= rhs;
	return res;
}

inline Vector3 operator*(const double& lhs, const Vector3& rhs)
{
	Vector3 res(rhs);
	res *= lhs;
	return res;
}


inline Vector3 operator*(const Vector3& lhs, const Vector3& rhs)
{
	Vector3 res(lhs);
	res *= rhs;
	return res;
}

inline Vector3 operator/(const Vector3& lhs, const double& rhs)
{
	Vector3 res(lhs);
	res /= rhs;
	return res;
}

}
#endif