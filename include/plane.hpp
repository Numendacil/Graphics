#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include "utils.hpp"
#include <cmath>

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D
{
public:
	Plane()
	{
		this->normal = {0.0, 0.0, 0.0};
		this->d = 0.0;
	}

	Plane(const Vector3f &normal, float d, Material *m) : Object3D(m)
	{
		this->normal = normal.normalized();
		this->d = d;
	}

	~Plane() override = default;

	bool intersect(const Ray &r, Hit &h, float tmin) const override
	{
		if (std::abs(Vector3f::dot(this->normal, r.getDirection())) < 1e-5)
			return false;
		float distance = -this->d + Vector3f::dot(r.getOrigin(), this->normal);
		float t = -distance / Vector3f::dot(this->normal, r.getDirection());
		if (t < 0)
			return false;
		if (t < tmin || t >= h.getT())
			return false;

		h.set(t, this->material, {r.GetAt(t), this->normal});
		return true;
	}

	HitSurface SamplePoint(double &pdf, RandomGenerator &rng) const override
	{
		pdf = -1.0f;
		return { Vector3f::ZERO, this->normal };
	}

protected:
	Vector3f normal;
	float d;
};

#endif //PLANE_H
