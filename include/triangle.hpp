#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

// TODO: implement this class and add more fields as necessary,
class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
		this->vertices[0] = a;
		this->vertices[1] = b;
		this->vertices[2] = c;
		this->normal = Vector3f::cross(b - a, c - a).normalized();
	}

	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, const Vector3f& normal, Material* m) : Object3D(m) {
		this->vertices[0] = a;
		this->vertices[1] = b;
		this->vertices[2] = c;
		this->normal = normal;
	}

	bool intersect( const Ray& ray,  Hit& hit , float tmin) const override {
        	Vector3f E1 = this->vertices[0] - this->vertices[1];
		Vector3f E2 = this->vertices[0] - this->vertices[2];
		Vector3f S = this->vertices[0] - ray.getOrigin();

		float det1 = Matrix3f(ray.getDirection(), E1, E2).determinant();
		if (abs(det1) < 1e-6)
			return false;
		
		float t = Matrix3f(S, E1, E2).determinant() / det1;
		if (t < 0)
			return false;
		if (t < tmin || t >= hit.getT())
			return false;

		float beta = Matrix3f(ray.getDirection(), S, E2).determinant() / det1;
		if (beta < 0 || beta > 1)
			return false;
		float gamma = Matrix3f(ray.getDirection(), E1, S).determinant() / det1;
		if (gamma < 0 || gamma > 1 || beta + gamma > 1)
			return false;

		hit.set(t, this->material, {ray.GetAt(t), this->normal});
		return true;
	}

	HitSurface SamplePoint(double &pdf, RandomGenerator &rng) const override
	{
		double area = Vector3f::cross(this->vertices[1] - this->vertices[0], this->vertices[2] - this->vertices[0]).length() / 2;
		pdf = 1.0f / area;
		double a = rng.GetUniformReal();
		double b = rng.GetUniformReal();
		if (a + b >= 1)
		{
			a = 1 - a;
			b = 1 - b;
		}
		Vector3f pos = (1 - a - b) *  this->vertices[0] + a *  this->vertices[1] + b *  this->vertices[2];
		return {pos, this->normal};
	}

protected:
	Vector3f normal;
	Vector3f vertices[3];

};

#endif //TRIANGLE_H
