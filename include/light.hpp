#ifndef LIGHT_H
#define LIGHT_H

#include <Vector3f.h>
#include "object3d.hpp"

class Light
{
public:
	Light() = default;

	virtual ~Light() = default;

	virtual Ray SampleRay(Vector3f &power, double& pdf, RandomGenerator& rng) const = 0;

	virtual Vector3f GetIllumin(const Vector3f &dir) const = 0;

	virtual bool intersect(const Ray &r, Hit &h, float tmin) const = 0;

};

class AreaLight : public Light
{
public:
	AreaLight() = delete;

	AreaLight(Object3D *object, const Vector3f &c)
	{
		this->object = object;
		power = c;
	}

	~AreaLight() override {delete this->object;}

	Ray SampleRay(Vector3f &power, double& pdf, RandomGenerator& rng) const override
	{
		power = this->power;
		HitSurface surface = this->object->SamplePoint(pdf, rng);
		Vector3f tangent = GetPerpendicular(surface.normal);
		Vector3f binormal = Vector3f::cross(surface.normal, tangent).normalized();

		double phi = 2 * M_PI * rng.GetUniformReal();
		double t = std::sqrt(rng.GetUniformReal());
		pdf *= t / M_PI;
		power *= t;
		Vector3f out = Vector3f(std::sqrt(1 - t * t) * std::cos(phi), std::sqrt(1 - t * t) * std::sin(phi), t);
		return {surface.position, RelToAbs(tangent, binormal, surface.normal, out)};
	}

	Vector3f GetIllumin(const Vector3f &dir) const override
	{
		return this->power;
	}

	bool intersect(const Ray &r, Hit &h, float tmin) const override
	{
		return this->object->intersect(r, h, tmin);
	}

private:
	Object3D* object;
	Vector3f power;
};

class PointLight : public Light
{
public:
	PointLight() = delete;

	PointLight(const Vector3f &p, const Vector3f &c)
	{
		this->position = p;
		this->power = c;
	}

	~PointLight() override = default;

	Ray SampleRay(Vector3f &power, double& pdf, RandomGenerator& rng) const override
	{
		power = this->power;
		pdf = 1.0f / (4 * M_PI);
		float phi = 2 * M_PI * rng.GetUniformReal();
		float z = 2 * rng.GetUniformReal() - 1;
		Vector3f dir(std::sqrt(1 - z * z) * std::cos(phi), std::sqrt(1 - z * z) * std::sin(phi), z);
		return {this->position, dir};
	}

	bool intersect(const Ray &r, Hit &h, float tmin) const override
	{
		return false;
	}

	Vector3f GetIllumin(const Vector3f &dir) const override
	{
		return this->power;
	}

private:
	Vector3f position;
	Vector3f power;
};

#endif // LIGHT_H
