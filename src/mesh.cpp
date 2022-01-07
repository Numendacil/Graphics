#include "mesh.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>

#include "plane.hpp"

bool Mesh::intersect(const Ray &r, Hit &h, float tmin) const
{

	// Optional: Change this brute force method into a faster one.
	bool result = false;
	for (int triId = 0; triId < (int)t.size(); ++triId)
	{
		TriangleIndex triIndex = t[triId];
		Triangle triangle(v[triIndex[0]],
						  v[triIndex[1]], v[triIndex[2]], n[triId], material);
		Plane plane(n[triId], Vector3f::dot(n[triId], v[triIndex[0]]), material);
		Hit h_test = h;
		if (plane.intersect(r, h_test, tmin))
			result |= triangle.intersect(r, h, tmin);
	}
	return result;
}

HitSurface Mesh::SamplePoint(double &pdf, RandomGenerator &rng) const
{
	int size = this->t.size();
	pdf = 1.0f / size;
	int idx = rng.GetUniformInt(0, size - 1);
	double objpdf;
	TriangleIndex triIndex = this->t[idx];
	Triangle triangle(this->v[triIndex[0]], this->v[triIndex[1]], this->v[triIndex[2]], this->n[idx], this->material);
	HitSurface s = triangle.SamplePoint(objpdf, rng);
	pdf *= objpdf;
	return s;
}

Mesh::Mesh(const char *filename, Material *material) : Object3D(material)
{

	// Optional: Use tiny obj loader to replace this simple one.
	std::ifstream f;
	f.open(filename);
	if (!f.is_open())
	{
		std::cout << "Cannot open " << filename << "\n";
		return;
	}
	std::string line;
	std::string vTok("v");
	std::string fTok("f");
	std::string texTok("vt");
	char bslash = '/', space = ' ';
	std::string tok;
	int texID;
	while (true)
	{
		std::getline(f, line);
		if (f.eof())
		{
			break;
		}
		if (line.size() < 3)
		{
			continue;
		}
		if (line.at(0) == '#')
		{
			continue;
		}
		std::stringstream ss(line);
		ss >> tok;
		if (tok == vTok)
		{
			Vector3f vec;
			ss >> vec[0] >> vec[1] >> vec[2];
			v.push_back(vec);
		}
		else if (tok == fTok)
		{
			if (line.find(bslash) != std::string::npos)
			{
				std::replace(line.begin(), line.end(), bslash, space);
				std::stringstream facess(line);
				TriangleIndex trig;
				facess >> tok;
				for (int ii = 0; ii < 3; ii++)
				{
					facess >> trig[ii] >> texID;
					trig[ii]--;
				}
				t.push_back(trig);
			}
			else
			{
				TriangleIndex trig;
				for (int ii = 0; ii < 3; ii++)
				{
					ss >> trig[ii];
					trig[ii]--;
				}
				t.push_back(trig);
			}
		}
		else if (tok == texTok)
		{
			Vector2f texcoord;
			ss >> texcoord[0];
			ss >> texcoord[1];
		}
	}
	computeNormal();

	f.close();
}

void Mesh::computeNormal()
{
	n.resize(t.size());
	for (int triId = 0; triId < (int)t.size(); ++triId)
	{
		TriangleIndex &triIndex = t[triId];
		Vector3f a = v[triIndex[1]] - v[triIndex[0]];
		Vector3f b = v[triIndex[2]] - v[triIndex[0]];
		n[triId] = Vector3f::cross(a, b).normalized();
	}
}
