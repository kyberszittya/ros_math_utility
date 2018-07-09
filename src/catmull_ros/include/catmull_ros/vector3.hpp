#ifndef CATMULL_ROS_VECTOR3_HPP
#define CATMULL_ROS_VECTOR3_HPP

namespace catmull_ros {
struct Vector3
{
	double x;
	double y;
	double z;

	Vector3() : x(0.0), y(0.0), z(0.0) {}
	Vector3(double x, double y, double z) : x(x), y(y), z(z) {}
	Vector3(const Vector3& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
	}

	Vector3 operator+=(const Vector3& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		return *this;
	}
	
	Vector3 operator-=(const Vector3& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		return *this;
	}

	Vector3 operator*=(const Vector3& rhs)
	{
		x *= rhs.x;
		y *= rhs.y;
		z *= rhs.z;
		return *this;
	}

	Vector3 operator+=(const double& rhs)
	{
		x += rhs;
		y += rhs;
		z += rhs;
		return *this;
	}

	Vector3 operator/=(const double& rhs)
	{
		x /= rhs;
		y /= rhs;
		z /= rhs;
		return *this;
	}

	Vector3 operator*=(const double& rhs)
	{
		x *= rhs;
		y *= rhs;
		z *= rhs;
		return *this;
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