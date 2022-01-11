#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement functions and add more fields as necessary

class Sphere : public Object3D
{
public:
	Sphere()
	{
		// unit ball at the center
		this->center = {0.0, 0.0, 0.0};
		this->radius = 1.0;
	}

	Sphere(const Vector3f &center, float radius, Material *material) : Object3D(material)
	{
		//
		this->center = center;
		this->radius = radius;
	}

	~Sphere() override = default;

	bool intersect(const Ray &r, Hit &h, float tmin) const override
	{
		//
		Vector3f l = this->center - r.getOrigin();
		float tp = Vector3f::dot(l, r.getDirection().normalized());
		float d_sqr = l.squaredLength() - tp * tp;

		if (d_sqr > this->radius * this->radius)
			return false;

		float t = 0.0;
		if (l.squaredLength() > this->radius * this->radius)
		{
			if (tp <= 0)
				return false;

			t = tp - sqrt(this->radius * this->radius - d_sqr);
		}
		else
			t = tp + sqrt(this->radius * this->radius - d_sqr);
		if (t < tmin || t >= h.getT())
			return false;

		h.set(t, this->material, {r.GetAt(t), (r.GetAt(t) - this->center).normalized()});
		return true;
	}

	HitSurface SamplePoint(double &pdf, RandomGenerator &rng) const override
	{
		pdf = 1.0f / (4 * M_PI * this->radius * this->radius);
		float phi = 2 * M_PI * rng.GetUniformReal();
		float z = 2 * rng.GetUniformReal() - 1;
		Vector3f normal(std::sqrt(1 - z * z) * std::cos(phi), std::sqrt(1 - z * z) * std::sin(phi), z);
		Vector3f pos = this->center + normal * this->radius;
		return {pos, center};
	}

protected:
	Vector3f center;
	float radius;
};

#endif
