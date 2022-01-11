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
		this->HasTexture = false;
	}

	Plane(const Vector3f &normal, float d, Material *m) : Object3D(m)
	{
		this->normal = normal.normalized();
		this->d = d;
		this->HasTexture = false;
	}

	~Plane() override = default;

	void SetTexCoord(const Vector3f &e_0, const Vector3f &e_1, const Vector3f& o)
	{
		this->origin = o + this->normal * (this->d - Vector3f::dot(this->normal, o));	// Make sure it's on the plane
		this->e[0] = e_0 - this->normal * Vector3f::dot(this->normal, e_0);
		this->e[1] = e_1 - this->normal * Vector3f::dot(this->normal, e_1);
		this->HasTexture = true;
	}

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

		if (this->HasTexture && this->material->HasTexture())
		{
			Vector3f pos = r.GetAt(t) - this->origin;
			float E = this->e[0].squaredLength() * this->e[1].squaredLength() - Vector3f::dot(this->e[0], this->e[1]) * Vector3f::dot(this->e[0], this->e[1]);
			float S1 = Vector3f::dot(pos, this->e[0]) * this->e[1].squaredLength() - Vector3f::dot(this->e[0], this->e[1]) * Vector3f::dot(pos, this->e[1]);
			float S2 = Vector3f::dot(pos, this->e[1]) * this->e[0].squaredLength() - Vector3f::dot(this->e[0], this->e[1]) * Vector3f::dot(pos, this->e[0]); 
			h.set(t, this->material, HitSurface(r.GetAt(t), this->normal, this->normal, Vector2f(S1 / E, S2 / E), true));
		}
		else
			h.set(t, this->material, HitSurface(r.GetAt(t), this->normal));
		return true;
	}

	HitSurface SamplePoint(double &pdf, RandomGenerator &rng) const override
	{
		pdf = -1.0f;
		return { Vector3f::ZERO, this->normal };
	}

protected:
	Vector3f normal;
	Vector3f e[2];	// Basis vector for texture, not necessarily perpendicular
	Vector3f origin;	// Origin coordinate for texture
	float d;
	bool HasTexture;
};

#endif //PLANE_H
