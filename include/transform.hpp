#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <vecmath.h>
#include "object3d.hpp"
#include "utils.hpp"


class Transform : public Object3D
{
public:
	Transform() {}

	Transform(const Matrix4f &m, Object3D *obj) : o(obj)
	{
		transform = m.inverse();
	}

	~Transform()
	{
	}

	virtual bool intersect(const Ray &r, Hit &h, float tmin) const
	{
		Vector3f trSource = transformPoint(transform, r.getOrigin());
		Vector3f trDirection = transformDirection(transform, r.getDirection());
		Ray tr(trSource, trDirection);
		bool inter = o->intersect(tr, h, tmin);
		if (inter)
		{
			h.set(h.getT(), h.getMaterial(), HitSurface(r.GetAt(h.getT()),
									 transformDirection(transform.transposed(), h.getSurface().normal).normalized(),
									 transformDirection(transform.transposed(), h.getSurface().geonormal).normalized(),
									 h.getSurface().texcoord,
									 h.getSurface().HasTexture));
		}
		return inter;
	}

	HitSurface SamplePoint(double &pdf, RandomGenerator &rng) const override
	{
		HitSurface s = o->SamplePoint(pdf, rng);
		return { transformPoint(this->transform.inverse(), s.position), transformDirection(this->transform.transposed(), s.normal).normalized() };
	}
protected:
	Object3D *o; //un-transformed object
	Matrix4f transform;
};

#endif //TRANSFORM_H
