#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

// TODO: implement this class and add more fields as necessary,
class Triangle : public Object3D
{

public:
	Triangle() = delete;

	// a b c are three vertex positions of the triangle
	Triangle(const Vector3f &a, const Vector3f &b, const Vector3f &c, Material *m) : Object3D(m)
	{
		this->vertices[0] = a;
		this->vertices[1] = b;
		this->vertices[2] = c;
		this->geonormal = Vector3f::cross(b - a, c - a).normalized();
		this->normal[0] = this->geonormal;
		this->normal[1] = this->geonormal;
		this->normal[2] = this->geonormal;
		this->HasTexture = false;
	}

	void SetNormal(const Vector3f &normal_0, const Vector3f &normal_1, const Vector3f &normal_2)
	{
		this->normal[0] = normal_0;
		this->normal[1] = normal_1;
		this->normal[2] = normal_2;
	}

	void SetTexCoord(const Vector2f &tex_0, const Vector2f &tex_1, const Vector2f &tex_2)
	{
		this->texcoord[0] = tex_0;
		this->texcoord[1] = tex_1;
		this->texcoord[2] = tex_2;
		this->HasTexture = true;
	}

	bool intersect(const Ray &ray, Hit &hit, float tmin) const override
	{
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

		Vector3f norm = (1 - beta - gamma) * this->normal[0] + beta * this->normal[1] + gamma * this->normal[2];
		Vector2f tex;
		if (this->HasTexture && this->material->HasTexture())
		{
			tex = (1 - beta - gamma) * this->texcoord[0] + beta * this->texcoord[1] + gamma * this->texcoord[2];
		}
		hit.set(t, this->material, HitSurface(ray.GetAt(t), norm, this->geonormal, tex, this->HasTexture && this->material->HasTexture()));
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
		Vector3f pos = (1 - a - b) * this->vertices[0] + a * this->vertices[1] + b * this->vertices[2];
		Vector3f norm = (1 - a - b) * this->normal[0] + a * this->normal[1] + b * this->normal[2];
		Vector2f tex;
		if (this->HasTexture && this->material->HasTexture())
			tex = (1 - a - b) * this->texcoord[0] + a * this->texcoord[1] + b * this->texcoord[2];
		return HitSurface(pos, norm, this->geonormal, tex, this->HasTexture && this->material->HasTexture());
	}

	Vector3f GetGeonormal() {return this->geonormal;}

protected:
	Vector3f normal[3];
	Vector3f geonormal;
	Vector3f vertices[3];
	Vector2f texcoord[3];
	bool HasTexture;
};

#endif //TRIANGLE_H
