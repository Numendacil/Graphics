#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include <vecmath.h>
#include <float.h>
#include <cmath>


class Camera {
public:
	Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH) {
		this->center = center;
		this->direction = direction.normalized();
		this->horizontal = Vector3f::cross(this->direction, up).normalized();
		this->up = Vector3f::cross(this->horizontal, this->direction);
		this->width = imgW;
		this->height = imgH;
	}

	// Generate rays for each screen-space coordinate
	virtual Ray generateRay(const Vector2f &point) = 0;
	virtual ~Camera() = default;

	int getWidth() const { return width; }
	int getHeight() const { return height; }

protected:
	// Extrinsic parameters
	Vector3f center;
	Vector3f direction;
	Vector3f up;
	Vector3f horizontal;
	// Intrinsic parameters
	int width;
	int height;
};

// TODO: Implement Perspective camera
// You can add new functions or variables whenever needed.
class PerspectiveCamera : public Camera {

public:
	PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
			const Vector3f &up, int imgW, int imgH, float angle) : Camera(center, direction, up, imgW, imgH) {
		// angle is in radian.
		this->fy = (float)imgH / (2.0f * tan(angle / 2.0f));
		this->fx = this->fy;
	}

	Ray generateRay(const Vector2f &point) override {
		// 
		Vector3f d = {(point.x() - this->width / 2.0f) / this->fx, (this->height / 2.0f - point.y()) / this->fy, 1.0f};
		d.normalize();
		Matrix3f rot(this->horizontal, -this->up, this->direction);
		return Ray(this->center, rot * d);
	}
protected:
	float fx;
	float fy;
};

#endif //CAMERA_H
