#ifndef HIT_H
#define HIT_H

#include <vecmath.h>
#include "ray.hpp"

class Material;

struct HitSurface
{
	Vector3f position;
	Vector3f normal;
};

class Hit
{
public:
	// constructors
	Hit()
	{
		this->material = nullptr;
		this->t = 1e5;
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
