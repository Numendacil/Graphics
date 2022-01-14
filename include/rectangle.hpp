#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

// Rectangle with face parallel to axis
class Rectangle : public Object3D
{

public:
	Rectangle() = delete;

	Rectangle(const Vector3f &a, const Vector3f &b, Material *m) : Object3D(m)
	{
		this->UpperRightFront = a;
		this->LowerLeftBehind = b;
		for (int i = 0; i < 3; i++)
			if (this->UpperRightFront[i] < this->LowerLeftBehind[i])
				std::swap(this->UpperRightFront[i], this->LowerLeftBehind[i]);
	}

	bool intersect(const Ray &ray, Hit &hit, float tmin) const override
	{
		// Woo's algorithm
		bool inside = true;
		enum {LEFT, RIGHT, MIDDLE} pos[3];
		Vector3f Candidate;
		Vector3f origin = ray.getOrigin(), dir = ray.getDirection().normalized();
		float length = ray.getDirection().length();
		for (int i = 0; i < 3; i++)
		{
			if (origin[i] < this->LowerLeftBehind[i] - std::max(0.0f, tmin * dir[i]))
			{
				pos[i] = LEFT;
				inside = false;
				Candidate[i] = this->LowerLeftBehind[i];
			}
			else if (origin[i] > this->UpperRightFront[i] + std::max(0.0f, -tmin * dir[i]))
			{
				pos[i] = RIGHT;
				inside = false;
				Candidate[i] = this->UpperRightFront[i];
			}
			else
			{
				pos[i] = MIDDLE;
				Candidate[i] = (dir[i] > 0)? this->UpperRightFront[i] : this->LowerLeftBehind[i];
			}
		}

		if (!inside)
		{
			float tmax = 0.0f;
			int maxIdx = 0;
			for (int i = 0; i < 3; i++)
			{
				if (pos[i] != MIDDLE && std::abs(dir[i]) > 1e-5)
				{
					if (tmax < (Candidate[i] - origin[i]) / dir[i])
					{
						tmax = (Candidate[i] - origin[i]) / dir[i];
						maxIdx = i;
					}
				}
			}
			if (tmax / length < tmin || tmax / length > hit.getT())
				return false;
			Vector3f position = ray.GetAt(tmax / length);
			Vector3f normal;
			for (int i = 0; i < 3; i++)
			{
				if (maxIdx != i)
				{
					if (position[i] < this->LowerLeftBehind[i] || position[i] > this->UpperRightFront[i])
						return false;
				}
				else
					normal[i] = (pos[i] == RIGHT)? -1 : 1;
			}

			if (this->material->HasTexture())
			{
				int face = maxIdx * 2 + ((normal[maxIdx] > 0)? 0 : 1);
				hit.set(tmax / length, this->material, HitSurface(position, normal, normal, MapToUV(position, face), true));
			}
			else
				hit.set(tmax / length, this->material, {position, normal});
			return true;
		}
		else
		{
			float t = INFINITY;
			int minIdx = 0;
			for (int i = 0; i < 3; i++)
			{
				if (std::abs(dir[i]) > 1e-5 && (Candidate[i] - origin[i]) / dir[i] >= 0)
				{
					if (t > (Candidate[i] - origin[i]) / dir[i])
					{
						t = (Candidate[i] - origin[i]) / dir[i];
						minIdx = i;
					}
				}
			}
			if (t / length < tmin || t / length > hit.getT())
				return false;
			Vector3f position = ray.GetAt(t / length);
			Vector3f normal;
			normal[minIdx] = (dir[minIdx] < 0)? -1 : 1;
			if (this->material->HasTexture())
			{
				int face = minIdx * 2 + ((normal[minIdx] > 0)? 0 : 1);
				hit.set(t / length, this->material, HitSurface(position, normal, normal, MapToUV(position, face), true));
			}
			else
				hit.set(t / length, this->material, {position, normal});
			return true;
		}
	}

	HitSurface SamplePoint(double &pdf, RandomGenerator &rng) const override
	{
		double area_xy, area_yz, area_zx;
		area_xy = (this->UpperRightFront[0] - this->LowerLeftBehind[0]) * (this->UpperRightFront[1] - this->LowerLeftBehind[1]);
		area_yz = (this->UpperRightFront[1] - this->LowerLeftBehind[1]) * (this->UpperRightFront[2] - this->LowerLeftBehind[2]);
		area_zx = (this->UpperRightFront[2] - this->LowerLeftBehind[2]) * (this->UpperRightFront[0] - this->LowerLeftBehind[0]);
		pdf = 1.0f / (area_xy + area_yz + area_zx) / 2.0f;

		double face = rng.GetUniformReal() * (area_xy + area_yz + area_zx);
		if (face < area_xy)
		{
			bool which = face < (area_xy / 2.0f);
			return {Vector3f(this->LowerLeftBehind[0] + rng.GetUniformReal() * (this->UpperRightFront[0] - this->LowerLeftBehind[0]),
					this->LowerLeftBehind[1] + rng.GetUniformReal() * (this->UpperRightFront[1] - this->LowerLeftBehind[1]),
					which? this->LowerLeftBehind[2] : this->UpperRightFront[2]), Vector3f(0, 0, which? -1 : 1)};
		}
		else if (face < area_xy + area_yz)
		{
			bool which = face - area_xy < (area_yz / 2.0f);
			return {Vector3f(which? this->LowerLeftBehind[0] : this->UpperRightFront[0],
					this->LowerLeftBehind[1] + rng.GetUniformReal() * (this->UpperRightFront[1] - this->LowerLeftBehind[1]),
					this->LowerLeftBehind[2] + rng.GetUniformReal() * (this->UpperRightFront[2] - this->LowerLeftBehind[2])), Vector3f(which? -1 : 1, 0, 0)};
		}
		else
		{
			bool which = face - area_xy - area_zx < (area_zx / 2.0f);
			return {Vector3f(this->LowerLeftBehind[0] + rng.GetUniformReal() * (this->UpperRightFront[0] - this->LowerLeftBehind[0]),
					which? this->LowerLeftBehind[1] : this->UpperRightFront[1],
					this->LowerLeftBehind[2] + rng.GetUniformReal() * (this->UpperRightFront[2] - this->LowerLeftBehind[2])), Vector3f(0, which? -1 : 1, 0)};
		}
	}

protected:
	Vector3f UpperRightFront;
	Vector3f LowerLeftBehind;

	Vector2f MapToUV(const Vector3f& Point, int face) const
	{
		Vector3f Size = this->UpperRightFront - this->LowerLeftBehind;
		Vector3f point = Point - this->LowerLeftBehind;
		Vector2f uv;
		assert(face >= 0 && face < 6);
		switch(face)
		{
		case 0:		// +x
			uv[0] = point[1] / (2.0f * (Size[0] + Size[1]));
			uv[1] = (point[2] + Size[0]) / (2.0f * Size[0] + Size[2]);
			break;
		case 1:		// -x
			uv[0] = (2.0f * Size[1] + Size[0] - point[1]) / (2.0f * (Size[0] + Size[1]));
			uv[1] = (point[2] + Size[0]) / (2.0f * Size[0] + Size[2]);
			break;
		case 2:		// +y
			uv[0] = (Size[1] + Size[0] - point[0]) / (2.0f * (Size[0] + Size[1]));
			uv[1] = (point[2] + Size[0]) / (2.0f * Size[0] + Size[2]);
			break;
		case 3:		// -y
			uv[0] = (2.0f * Size[1] + Size[0] + point[0]) / (2.0f * (Size[0] + Size[1]));
			uv[1] = (point[2] + Size[0]) / (2.0f * Size[0] + Size[2]);
			break;
		case 4:		// +z
			uv[0] = point[1] / (2.0f * (Size[0] + Size[1]));
			uv[1] = 1 - point[0] / (2.0f * Size[0] + Size[2]);
			break;
		case 5:		// -z
			uv[0] = point[1] / (2.0f * (Size[0] + Size[1]));
			uv[1] = point[0] / (2.0f * Size[0] + Size[2]);
			break;
		}
		return uv;
	}
};

#endif //RECTANGLE_H
