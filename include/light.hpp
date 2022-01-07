#ifndef LIGHT_H
#define LIGHT_H

#include <Vector3f.h>
#include "object3d.hpp"

class Light
{
public:
	Light() = default;

	virtual ~Light() = default;

	virtual Ray SampleRay(Vector3f &color, double& pdf, RandomGenerator& rng) const = 0;

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
		color = c;
	}

	~AreaLight() override = default;

	Ray SampleRay(Vector3f &color, double& pdf, RandomGenerator& rng) const override
	{
		color = this->color;
		HitSurface surface = this->object->SamplePoint(pdf, rng);
		Vector3f tangent = GetPerpendicular(surface.normal);
		Vector3f binormal = Vector3f::cross(surface.normal, tangent).normalized();

		double phi = rng.GetUniformReal(0, 2 * M_PI);
		double theta = std::acos(rng.GetUniformReal());
		pdf *= std::cos(theta) / M_PI;
		color *= std::cos(theta);
		Vector3f out = Vector3f(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
		return {surface.position, RelToAbs(tangent, binormal, surface.normal, out)};
	}

	Vector3f GetIllumin(const Vector3f &dir) const
	{
		return this->color;
	}

	bool intersect(const Ray &r, Hit &h, float tmin) const
	{
		return this->object->intersect(r, h, tmin);
	}

private:
	Object3D* object;
	Vector3f color;
};

class PointLight : public Light
{
public:
	PointLight() = delete;

	PointLight(const Vector3f &p, const Vector3f &c)
	{
		this->position = p;
		this->color = c;
	}

	~PointLight() override = default;

	Ray SampleRay(Vector3f &color, double& pdf, RandomGenerator& rng) const override
	{
		color = this->color;
		pdf = 1.0f / (4 * M_PI);
		float phi = rng.GetUniformReal() * 2 * M_PI;
		float theta = std::acos(2 * rng.GetUniformReal() - 1);
		Vector3f dir(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
		return {this->position, dir};
	}

	bool intersect(const Ray &r, Hit &h, float tmin) const
	{
		return false;
	}

	Vector3f GetIllumin(const Vector3f &dir) const
	{
		return this->color;
	}

private:
	Vector3f position;
	Vector3f color;
};

#endif // LIGHT_H
