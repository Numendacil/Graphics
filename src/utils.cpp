#include "utils.hpp"

// transforms a 3D point using a matrix, returning a 3D point
Vector3f transformPoint(const Matrix4f &mat, const Vector3f &point)
{
	return (mat * Vector4f(point, 1)).xyz();
}

// transform a 3D direction using a matrix, returning a direction
Vector3f transformDirection(const Matrix4f &mat, const Vector3f &dir)
{
	return (mat * Vector4f(dir, 0)).xyz();
}

// Switching vector between two frames
Vector3f AbsToRel(const Vector3f& axis_x, 
				const Vector3f& axis_y, 
				const Vector3f& axis_z, 
				const Vector3f &dir)
{
	return Vector3f(Vector3f::dot(dir, axis_x), 
			Vector3f::dot(dir, axis_y), 
			Vector3f::dot(dir, axis_z));
}

Vector3f RelToAbs(const Vector3f& axis_x, 
				const Vector3f& axis_y, 
				const Vector3f& axis_z, 
				const Vector3f &dir)
{
	return dir[0] * axis_x + dir[1] * axis_y + dir[2] * axis_z;
}

Vector3f GetPerpendicular(const Vector3f& v)	// Return some normalized vector perpendicular to v
{
	if (std::abs(v[0]) <= std::abs(v[1]) && std::abs(v[0]) <= std::abs(v[2]))
		return Vector3f(0, v[2], -v[1]).normalized();
	else if (std::abs(v[1]) <= std::abs(v[0]) && std::abs(v[1]) <= std::abs(v[2]))
		return Vector3f(v[2], 0,  -v[0]).normalized();
	else
		return Vector3f(v[1], -v[0], 0).normalized();
}

// Calculate Reflection
Vector3f Reflect(const Vector3f& v, const Vector3f& n)
{
	return -v + 2 * Vector3f::dot(v, n) * n;
}

Vector3f Refract(const Vector3f& v, const Vector3f& n, float ior1, float ior2)
{
	float mu = ior1 / ior2;
	float co = Vector3f::dot(v, n);
	assert(co >= 0);

	if (mu * mu * (1 - co * co) >= 1)
		return Vector3f::ZERO;
	else
		return mu * (n * co - v) - std::sqrt(1 - mu * mu * (1 - co * co)) * n;
}

bool CheckValid(const Vector3f& v)
{
	return !(std::isnan(v[0]) || std::isnan(v[1]) || std::isnan(v[2]) || v[0] < 0 || v[1] < 0 || v[2] < 0 || std::isinf(v[0]) || std::isinf(v[1]) || std::isinf(v[2]));
}