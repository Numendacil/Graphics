#ifndef MESH_H
#define MESH_H

#include <vector>
#include <map>
#include "object3d.hpp"
#include "triangle.hpp"
#include "rectangle.hpp"
#include "Vector2f.h"
#include "Vector3f.h"

class Mesh : public Object3D
{

public:
	Mesh(const char *filename, Material *m);
	~Mesh() {if (BoundingBox != nullptr) delete BoundingBox; for (auto& p : MeshMaterial) delete p.second;}

	struct TriangleIndex
	{
		int vIdx[3] = {};
		int nIdx[3] = {};
		int texIdx[3] = {};
		bool hasNormal = false;
		bool hasTexture = false;
		Material* material;
	};
	bool intersect(const Ray &r, Hit &h, float tmin) const override;
	HitSurface SamplePoint(double &pdf, RandomGenerator &rng) const override;

private:
	void parseMtl(const string& filename);
	
	std::vector<Vector3f> v;
	std::vector<TriangleIndex> t;
	std::vector<Vector3f> n;
	std::vector<Vector2f> texcoord;
	Rectangle* BoundingBox;
	std::map<std::string, Material*> MeshMaterial;
};

#endif
