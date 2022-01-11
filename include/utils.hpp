#ifndef UTILS_H
#define UTILS_H
#include <vecmath.h>
#include <random>
#include "third party/log.h"
#define SHOWVEC(x) logging::INFO(#x + std::to_string((x)[0]) + ", " + std::to_string((x)[1]) + ", " + std::to_string((x)[2]))
class RandomGenerator
{
private:
	std::mt19937 mt;
	unsigned seed;
public:
	RandomGenerator() { std::random_device rd; seed = rd(); mt.seed(seed);}
	RandomGenerator(unsigned sd) : mt(sd), seed(sd){}
	void SetSeed(unsigned sd) {this->mt.seed(sd); this->seed = sd;}
	unsigned GetSeed() {return this->seed;}
	std::vector<double> GetUniformReal(double min, double max, int num)
	{
		std::uniform_real_distribution<double> dist(min, max);
		std::vector<double> ret;
		for (int i = 0; i < num; i++)
			ret.push_back(dist(this->mt));
		return ret;
	}
	double GetUniformReal(double min = 0.0f, double max = 1.0f)
	{
		std::uniform_real_distribution<double> dist(min, max);
		return dist(this->mt);
	}
	std::vector<int> GetUniformInt(int min, int max, int num)
	{
		std::uniform_int_distribution<int> dist(min, max);
		std::vector<int> ret;
		for (int i = 0; i < num; i++)
			ret.push_back(dist(this->mt));
		return ret;
	}
	int GetUniformInt(int min = 0, int max = 10)
	{
		std::uniform_int_distribution<int> dist(min, max);
		return dist(this->mt);
	}
};

// transform a 3D point using a matrix, returning a point
Vector3f transformPoint(const Matrix4f &mat, const Vector3f &point);

// transform a 3D direction using a matrix, returning a direction
Vector3f transformDirection(const Matrix4f &mat, const Vector3f &dir);

// Switching vector between two frames
Vector3f AbsToRel(const Vector3f& axis_x, 
				const Vector3f& axis_y, 
				const Vector3f& axis_z, 
				const Vector3f &dir);

Vector3f RelToAbs(const Vector3f& axis_x, 
				const Vector3f& axis_y, 
				const Vector3f& axis_z, 
				const Vector3f &dir);

// Return some normalized vector perpendicular to v
Vector3f GetPerpendicular(const Vector3f& v);

// Calculate Reflection
Vector3f Reflect(const Vector3f& v, const Vector3f& n);

// Calculate Refraction, n points from 2 to 1, v in 1 and dot(n, v)
Vector3f Refract(const Vector3f& v, const Vector3f& n, float ior1, float ior2);

bool CheckValid(const Vector3f& v);


#endif