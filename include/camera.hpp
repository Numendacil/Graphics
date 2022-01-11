#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include <vecmath.h>
#include <float.h>
#include <cmath>

class Camera
{
public:
	Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH, float gamma)
	{
		this->center = center;
		this->direction = direction.normalized();
		this->horizontal = Vector3f::cross(this->direction, up).normalized();
		this->up = Vector3f::cross(this->horizontal, this->direction).normalized();
		this->width = imgW;
		this->height = imgH;
		this->gamma = gamma;
	}

	// Generate rays for each screen-space coordinate
	virtual Ray SampleRay(int x, int y, RandomGenerator& rng) const = 0;
	virtual ~Camera() = default;

	int getWidth() const { return width; }
	int getHeight() const { return height; }
	float getGamma() const { return this->gamma; }

protected:
	// Extrinsic parameters
	Vector3f center;
	Vector3f direction;
	Vector3f up;
	Vector3f horizontal;
	// Intrinsic parameters
	int width;
	int height;
	float gamma;
};

// TODO: Implement Perspective camera
// You can add new functions or variables whenever needed.
class PerspectiveCamera : public Camera
{

public:
	PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
			  const Vector3f &up, int imgW, int imgH, float gamma, float angle) : Camera(center, direction, up, imgW, imgH, gamma)
	{
		// angle is in radian.
		this->fy = (float)imgH / (2.0f * tan(angle / 2.0f));
		this->fx = this->fy;
	}

	Ray SampleRay(int x, int y, RandomGenerator& rng) const override
	{
		float delta_x = rng.GetUniformReal() - 0.5;
		float delta_y = rng.GetUniformReal() - 0.5;
		Vector3f d = {(x + delta_x - this->width / 2.0f) / this->fx, (this->height / 2.0f - y - delta_y) / this->fy, 1.0f};
		d.normalize();
		Matrix3f rot(this->horizontal, -this->up, this->direction);
		return Ray(this->center, rot * d);
	}

protected:
	float fx;
	float fy;
};

class LensCamera : public Camera
{

public:
	LensCamera(const Vector3f &center, const Vector3f &direction,
			  const Vector3f &up, int imgW, int imgH, float gamma, float angle, float aperture, float focallength) : Camera(center, direction, up, imgW, imgH, gamma)
	{
		// angle is in radian.
		this->fy = (float)imgH / (2.0f * tan(angle / 2.0f));
		this->fx = this->fy;
		this->aperture = aperture;
		this->FocalLength = focallength;
	}

	Ray SampleRay(int x, int y, RandomGenerator& rng) const override
	{
		float u, v;
		do
		{
			u = 2 * rng.GetUniformReal() - 1;
			v = 2 * rng.GetUniformReal() - 1;
		}while (u * u + v * v > 1);	// Simple reject sampling
		Vector3f origin = this->center + u * (this->aperture / 2.0f) * this->up + v * (this->aperture / 2.0f) * this->horizontal;
		Vector3f d = Vector3f((x - this->width / 2.0f) / this->fx, (this->height / 2.0f - y) / this->fy, 1.0f).normalized() * this->FocalLength;
		d = (d + this->center - origin).normalized() ;
		Matrix3f rot(this->horizontal, -this->up, this->direction);
		return Ray(origin, (rot * d).normalized());
	}

protected:
	float fx;
	float fy;
	float aperture;
	float FocalLength;
};

#endif //CAMERA_H
