#ifndef HIT_H
#define HIT_H

#include <vecmath.h>
#include "ray.hpp"
#include "utils.hpp"

class Material;

struct HitSurface
{
	Vector3f position;
	Vector3f normal;
	Vector3f geonormal;
	Vector2f texcoord;
	bool HasTexture;
	HitSurface(){}
	HitSurface(const Vector3f& pos, const Vector3f& norm, const Vector3f& geonorm = Vector3f::ZERO, const Vector2f& texcoord = Vector2f::ZERO, bool flag = false)
	{
		this->position = pos;
		this->normal = norm;
		this->geonormal = (geonorm == Vector3f::ZERO)? norm : geonorm;
		this->texcoord = texcoord;
		this->HasTexture = flag;
	}
};

class Hit
{
public:
	// constructors
	Hit()
	{
		this->material = nullptr;
		this->t = INFINITY;
	}

	Hit(float _t, Material *m, const HitSurface &s)
	{
		this->t = _t;
		this->material = m;
		this->surface = s;
	}

	Hit(const Hit &h)
	{
		this->t = h.t;
		this->material = h.material;
		this->surface = h.surface;
	}

	// destructor
	~Hit() = default;

	float getT() const
	{
		return this->t;
	}

	Material *getMaterial() const
	{
		return this->material;
	}

	const HitSurface &getSurface() const
	{
		return this->surface;
	}

	void set(float _t, Material *m, const HitSurface &s)
	{
		this->t = _t;
		this->material = m;
		this->surface = s;
	}

private:
	float t;
	Material *material;
	HitSurface surface;
};


#endif // HIT_H
