#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include "utils.hpp"

enum MaterialType {DIFFUSE, SPECULAR};
enum TransportMode {LIGHT, CAMERA};	// Non-symmetric Scattering


class Material	// The coordinates are in the material reference frame
{
protected:
	MaterialType type;

public:
	Material(MaterialType type) {this->type = type;}
	Material() {this->type = DIFFUSE;}

	MaterialType GetType() const {return type;};

	virtual Vector3f Shade(const Vector3f& in, const Vector3f& out, const TransportMode mode) const = 0;
	virtual Vector3f SampleOutDir(const Vector3f& in, Vector3f& out, const TransportMode mode, double& pdf, RandomGenerator& rng) const = 0;
};

class Lambert : public Material
{
protected:
	Vector3f color;
public:
	Lambert(const Vector3f& color) : Material(DIFFUSE) {this->color = color;}
	Vector3f Shade(const Vector3f& in, const Vector3f& out, const TransportMode mode) const override
	{
		// cos(r, n) = r[2]
		if (in[2] < 0 || out[2] < 0)
			return Vector3f::ZERO;
		return this->color / M_PI;
	}
	Vector3f SampleOutDir(const Vector3f& in, Vector3f& out, const TransportMode mode, double& pdf, RandomGenerator& rng) const override
	{
		double phi = rng.GetUniformReal(0, 2 * M_PI);
		double theta = std::acos(rng.GetUniformReal());
		pdf = std::cos(theta) / M_PI;
		out = Vector3f(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
		return Shade(in, out, mode);
	}
};

class Phong : public Material
{
protected:
	Vector3f diffuseColor;
	Vector3f specularColor;
	float diff, spec;	// diff + spec < 1, probabilities for diffusion and speculation
	float shininess;
public:
	Phong(const Vector3f &d_color, const Vector3f &s_color, float diff, float spec, float s) : Material(DIFFUSE)
	{
		this->diffuseColor = d_color;
		this->specularColor = s_color;
		this->diff = diff;
		this->spec = spec;
		this->shininess = s;
	}
	Vector3f Shade(const Vector3f& in, const Vector3f& out, const TransportMode mode) const override
	{
		if (mode == TransportMode::LIGHT)
		{
			float co_d = in[2];
			float co_s = Vector3f::dot(out, Reflect(in, Vector3f(0, 0, 1)));
			co_d = (co_d > 0) ? co_d : 0;
			co_s = (co_s > 0) ? co_s : 0;
			return this->diffuseColor * co_d + this->specularColor * pow(co_s, this->shininess);
		}
		else
		{
			float co_d = out[2];
			float co_s = Vector3f::dot(in, Reflect(out, Vector3f(0, 0, 1)));
			co_d = (co_d > 0) ? co_d : 0;
			co_s = (co_s > 0) ? co_s : 0;
			return this->diffuseColor * co_d + this->specularColor * pow(co_s, this->shininess);
		}
	}
	Vector3f SampleOutDir(const Vector3f& in, Vector3f& out, const TransportMode mode, double& pdf, RandomGenerator& rng) const override
	{
		Vector3f ref = this->diffuseColor * this->diff + this->specularColor * this->spec;
		float prob_r = std::max(ref[0], std::max(ref[1], ref[2]));
		Vector3f pdiff = this->diffuseColor * this->diff;
		float prob_d = prob_r * (pdiff[0] + pdiff[1] + pdiff[2]) / (ref[0] + ref[1] + ref[2]);
		double rnd = rng.GetUniformReal();
		if (rnd < prob_d)	// Diffuse
		{
			double phi = rng.GetUniformReal(0, 2 * M_PI);
			double theta = std::acos(rng.GetUniformReal());
			pdf = std::cos(theta) / M_PI;
			out = Vector3f(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
			return Shade(in, out, mode);
		}
		else if (rnd < prob_r)	// Specular
		{
			out = Reflect(in, Vector3f(0, 0, 1));
			pdf = 1;
			return Shade(in, out, mode);
		}
		else // Absorbed
		{
			pdf = 1;
			return Vector3f::ZERO;
		}
	}
};

class Mirror : public Material
{
protected:
	Vector3f color;

public:
	Mirror(const Vector3f& c) : Material(SPECULAR)
	{
		this->color = c;
	}

	Vector3f Shade(const Vector3f& in, const Vector3f& out, const TransportMode mode) const override
	{
		return Vector3f::ZERO;
	}

	Vector3f SampleOutDir(const Vector3f& in, Vector3f& out, const TransportMode mode, double& pdf, RandomGenerator& rng) const override
	{
		out = Reflect(in, Vector3f(0, 0, 1));
		pdf = 1;
		return color / std::abs(out[2]);
	}
};



#endif // MATERIAL_H
