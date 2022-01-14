#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include "utils.hpp"
#include "image.hpp"

enum RefType {DIFFUSE, SPECULAR};
enum TransportMode {LIGHT, CAMERA};	// Non-symmetric Scattering


class Material	// The coordinates are in the material reference frame
{
protected:
	Image* texture = nullptr;
public:
	Material() {}
	virtual ~Material(){ delete texture;}

	virtual Vector3f Shade(const Vector3f& in, const Vector3f& out, const TransportMode mode) const = 0;
	virtual bool HasTexture() const { return this->texture != nullptr;}
	virtual Vector3f GetTexture(const Vector2f& texcoord) const
	{
		if (this->texture == nullptr)
			return Vector3f::ZERO;
		int x = (texcoord[0] - std::floor(texcoord[0])) * this->texture->Width();
		x = (x >= this->texture->Width())? this->texture->Width() - 1 : x;
		int y = (texcoord[1] - std::floor(texcoord[1])) * this->texture->Height();
		y = (y >=  this->texture->Height())? this->texture->Height() - 1 : y;
		return this->texture->GetPixel(x, y);
	}
	virtual Vector3f SampleOutDir(const Vector3f& in, Vector3f& out, const TransportMode mode, double& pdf, RefType& type, RandomGenerator& rng) const = 0;
};

class Generic : public Material		// Generic model for material used in .mtl file 
{
protected:
	Vector3f Ka;	// ambient color
	Vector3f Kd;	// diffuse color
	Vector3f Ks;	// specular color
	float Ns;		// specular highlights
	float Ni;	// optical density
	float d;	// dissolve

public:
	Generic(const Vector3f& ka, const Vector3f& kd, const Vector3f& ks, int ns, float ni, float d)
	{
		this->Ka = ka;
		this->Kd = kd;
		this->Ks = ks;
		this->Ns = ns;
		this->Ni = ni;
		this->d = d;
	}

	Generic(const Vector3f& ka, const Vector3f& kd, const Vector3f& ks, int ns, float ni, float d, const std::string& filename)
	{
		this->Ka = ka;
		this->Kd = kd;
		this->Ks = ks;
		this->Ns = ns;
		this->Ni = ni;
		this->d = d;
		this->texture = Image::LoadTGA(filename.c_str());
	}

	Vector3f Shade(const Vector3f& in, const Vector3f& out, const TransportMode mode) const override
	{
		if (in[2] * out[2] < 0)
			return Vector3f::ZERO;
		float co_s = Vector3f::dot(out, Reflect(in, Vector3f(0, 0, 1)));
		co_s = (co_s > 0) ? co_s : 0;
		// Modified Phong model
		return this->Kd / M_PI + this->Ks * pow(co_s, this->Ns) * (2 + this->Ns) / (2 * M_PI) ;
	}

	Vector3f SampleOutDir(const Vector3f& in, Vector3f& out, const TransportMode mode, double& pdf, RefType& type, RandomGenerator& rng) const override
	{
		if (rng.GetUniformReal() < d)	// Reflect
		{
			Vector3f ref = this->Kd + this->Ks;
			float prob_r = std::max(ref[0], std::max(ref[1], ref[2]));
			prob_r = (prob_r > 1.0f)? 1.0f : prob_r;
			float prob_d = prob_r * (this->Kd[0] + this->Kd[1] + this->Kd[2]) / (ref[0] + ref[1] + ref[2]);
			double rnd = rng.GetUniformReal();
			if (rnd < prob_d)	// diffuse
			{
				type = RefType::DIFFUSE;
				double phi = 2 * M_PI * rng.GetUniformReal();
				double t = std::sqrt(rng.GetUniformReal());
				pdf = t * prob_r / M_PI;
				out = Vector3f(std::sqrt(1 - t * t) * std::cos(phi), std::sqrt(1 - t * t) * std::sin(phi), t);
				return this->Kd / M_PI;
			}
			else if (rnd < prob_r)	// Specular reflection
			{
				type = RefType::SPECULAR;
				pdf = prob_r;
				out = Reflect(in, Vector3f(0, 0, 1));
				return this->Ks / (std::abs(out[2]) + 1e-5);
			}
			else // Absorbed
			{
				type = RefType::DIFFUSE;
				pdf = 1.0f;
				return Vector3f::ZERO;
			}
		}
		else // Refract
		{
			type = RefType::SPECULAR;
			float scale = this->Ni * this->Ni;	// Non-symmetry due to refraction energy in different medium
			if (in[2] >= 0) // Into the medium
			{
				out = Refract(in, Vector3f(0, 0, 1), 1.0f, this->Ni);
				scale = 1.0f / scale;
			}
			else
			{
				out = Refract(in, Vector3f(0, 0, -1), this->Ni, 1.0f);
			}
			pdf = 1.0f;
			if (out == Vector3f::ZERO)
				return Vector3f::ZERO;
			
			if (mode == TransportMode::CAMERA)
				return scale * Vector3f(1, 1, 1) / (std::abs(out[2]) + 1e-5);
			else
				return Vector3f(1, 1, 1) / (std::abs(out[2]) + 1e-5);
		}
	}

};

class Lambert : public Material
{
protected:
	Vector3f color;
public:
	Lambert(const Vector3f& color) 
	{
		this->color = color; 
		this->texture = nullptr;
	}
	Lambert(const Vector3f& color, const std::string& filename)
	{
		this->color = color; 
		this->texture = Image::LoadTGA(filename.c_str());
	}
	Vector3f Shade(const Vector3f& in, const Vector3f& out, const TransportMode mode) const override
	{
		// cos(r, n) = r[2]
		if (in[2] * out[2] < 0)
			return Vector3f::ZERO;
		return this->color / M_PI;
	}
	Vector3f SampleOutDir(const Vector3f& in, Vector3f& out, const TransportMode mode, double& pdf, RefType& type, RandomGenerator& rng) const override
	{
		double phi = 2 * M_PI * rng.GetUniformReal();
		double t = std::sqrt(rng.GetUniformReal());
		pdf = t / M_PI;
		type = RefType::DIFFUSE;
		out = Vector3f(std::sqrt(1 - t * t) * std::cos(phi), std::sqrt(1 - t * t) * std::sin(phi), t);
		return Shade(in, out, mode);
	}
};

class Phong : public Material
{
protected:
	Vector3f diffuseColor;
	Vector3f specularColor;
	float shininess;
public:
	Phong(const Vector3f &d_color, const Vector3f &s_color, float s)
	{
		this->diffuseColor = d_color;
		this->specularColor = s_color;
		this->shininess = s;
		this->texture = nullptr;
	}
	Phong(const Vector3f &d_color, const Vector3f &s_color, float s, const std::string& filename)
	{
		this->diffuseColor = d_color;
		this->specularColor = s_color;
		this->shininess = s;
		this->texture = Image::LoadTGA(filename.c_str());
	}
	Vector3f Shade(const Vector3f& in, const Vector3f& out, const TransportMode mode) const override
	{
		if (in[2] * out[2] < 0)
			return Vector3f::ZERO;
		float co_s = Vector3f::dot(out, Reflect(in, Vector3f(0, 0, 1)));
		co_s = (co_s > 0) ? co_s : 0;
		// Modified Phong model
		return this->diffuseColor / M_PI + this->specularColor * pow(co_s, this->shininess) * (2 + this->shininess) / (2 * M_PI) ;
	}
	Vector3f SampleOutDir(const Vector3f& in, Vector3f& out, const TransportMode mode, double& pdf, RefType& type, RandomGenerator& rng) const override
	{
		Vector3f ref = this->diffuseColor + this->specularColor;
		type = RefType::DIFFUSE;
		float prob_r = std::max(ref[0], std::max(ref[1], ref[2]));
		prob_r = (prob_r > 1.0f)? 1.0f : prob_r;
		Vector3f pdiff = this->diffuseColor;
		float prob_d = prob_r * (pdiff[0] + pdiff[1] + pdiff[2]) / (ref[0] + ref[1] + ref[2]);
		double rnd = rng.GetUniformReal();

		if (rnd < prob_d)	// Diffuse sampling
		{
			double phi = 2 * M_PI * rng.GetUniformReal();
			double t = std::sqrt(rng.GetUniformReal());
			pdf = t * prob_r / M_PI;
			out = Vector3f(std::sqrt(1 - t * t) * std::cos(phi), std::sqrt(1 - t * t) * std::sin(phi), t);
			return this->diffuseColor / M_PI;
		}

		else if (rnd < prob_r)	// Specular sampling
		{
			Vector3f ref = Reflect(in, Vector3f(0, 0, 1));
			Vector3f tangent = GetPerpendicular(ref);
			Vector3f binormal = Vector3f::cross(ref, tangent).normalized();

			double phi = 2 * M_PI * rng.GetUniformReal();
			double t = std::pow(std::sqrt(rng.GetUniformReal()), 1.0f / (1.0f + this->shininess));
			out = RelToAbs(tangent, binormal, ref, Vector3f(std::sqrt(1 - t * t) * std::cos(phi), std::sqrt(1 - t * t) * std::sin(phi), t));
			pdf = prob_r * (this->shininess + 2.0f) * std::pow(t, this->shininess) / (2.0f * M_PI);
			return Shade(in, out, mode) - this->diffuseColor / M_PI;	// The specular part
		}

		else // Absorbed
		{
			pdf = 1.0f;
			return Vector3f::ZERO;
		}
	}
};

class Mirror : public Material
{
protected:
	Vector3f color;

public:
	Mirror(const Vector3f& c)
	{
		this->color = c;
		this->texture = nullptr;
	}
	Mirror(const Vector3f& c, const std::string& filename)
	{
		this->color = c;
		this->texture = Image::LoadTGA(filename.c_str());
	}

	Vector3f Shade(const Vector3f& in, const Vector3f& out, const TransportMode mode) const override
	{
		return Vector3f::ZERO;
	}

	Vector3f SampleOutDir(const Vector3f& in, Vector3f& out, const TransportMode mode, double& pdf, RefType& type, RandomGenerator& rng) const override
	{
		out = Reflect(in, Vector3f(0, 0, 1));
		pdf = 1;
		type = RefType::SPECULAR;
		return this->color / (std::abs(out[2]) + 1e-5);
	}
};

class Transparent : public Material
{
protected:
	Vector3f color;
	float IoR;	// Index of Refraction

public:
	Transparent(const Vector3f& c, float ior)
	{
		this->color = c;
		this->IoR = ior;
		this->texture = nullptr;
	}

	Transparent(const Vector3f& c, float ior, const std::string& filename)
	{
		this->color = c;
		this->IoR = ior;
		this->texture = Image::LoadTGA(filename.c_str());
	}

	Vector3f Shade(const Vector3f& in, const Vector3f& out, const TransportMode mode) const override
	{
		return Vector3f::ZERO;
	}

	Vector3f SampleOutDir(const Vector3f& in, Vector3f& out, const TransportMode mode, double& pdf, RefType& type, RandomGenerator& rng) const override
	{
		Vector3f reflect = Reflect(in, Vector3f(0, 0, 1));
		Vector3f refract;
		float scale = this->IoR * this->IoR;	// Non-symmetry due to refraction energy in different medium 
		if (in[2] >= 0)	// Into the medium
		{
			refract = Refract(in, Vector3f(0, 0, 1), 1.0f, this->IoR);
			scale = 1.0f / scale;
		}
		else
		{
			refract = Refract(in, Vector3f(0, 0, -1), this->IoR, 1.0f);
		}
		
		type = RefType::SPECULAR;
		float co_i = std::abs(reflect[2]), co_t = std::abs(refract[2]);
		float R_s = (co_i - this->IoR * co_t) * (co_i - this->IoR * co_t) / ((co_i + this->IoR * co_t) * (co_i + this->IoR * co_t));
		float R_p = (co_t - this->IoR * co_i) * (co_t - this->IoR * co_i) / ((co_t + this->IoR * co_i) * (co_t + this->IoR * co_i));
		if (rng.GetUniformReal() < (R_s + R_p) / 2.0f)	// Reflect
		{
			out = reflect;
			pdf = 1.0f;
			return this->color / (co_i + 1e-5);
		}
		else
		{
			out = refract;
			pdf = 1.0f;
			if (mode == TransportMode::CAMERA)
				return scale * this->color / (co_t + 1e-5);
			else
				return this->color / (co_t + 1e-5);
		}
	}
};



#endif // MATERIAL_H
