#ifndef MESH_H
#define MESH_H

#include <vector>
#include <map>
#include <numeric>
#include "object3d.hpp"
#include "triangle.hpp"
#include "Vector2f.h"
#include "Vector3f.h"

class BBox : Object3D
{
public:
	BBox() = delete;

	BBox(const Vector3f &a, const Vector3f &b) : Object3D(nullptr)
	{
		this->UpperRightFront = a;
		this->LowerLeftBehind = b;
		for (int i = 0; i < 3; i++)
			if (this->UpperRightFront[i] < this->LowerLeftBehind[i])
				std::swap(this->UpperRightFront[i], this->LowerLeftBehind[i]);
	}

	bool intersect(const Ray &ray, Hit &hit, float tmin) const override
	{
		tmin = 0.0f;	// tmin should not work for bounding box

		// Woo's algorithm
		bool inside = true;
		enum {LEFT, RIGHT, MIDDLE} pos[3];
		Vector3f Candidate;
		Vector3f origin = ray.getOrigin(), dir = ray.getDirection().normalized();
		float length = ray.getDirection().length();
		for (int i = 0; i < 3; i++)
		{
			if (origin[i] < this->LowerLeftBehind[i])
			{
				pos[i] = LEFT;
				inside = false;
				Candidate[i] = this->LowerLeftBehind[i];
			}
			else if (origin[i] > this->UpperRightFront[i])
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
				if (pos[i] != MIDDLE && std::abs(dir[i]) > 1e-6)
				{
					if (tmax < (Candidate[i] - origin[i]) / dir[i])
					{
						tmax = (Candidate[i] - origin[i]) / dir[i];
						maxIdx = i;
					}
				}
			}
			if (tmax  < 0 || tmax / length > hit.getT())
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
			hit.set(tmax / length, nullptr, {position, normal});
			return true;
		}
		else
		{
			hit.set(0, nullptr, {Vector3f::ZERO, Vector3f::ZERO});
			return true;
		}
	}

	HitSurface SamplePoint(double &pdf, RandomGenerator &rng) const override
	{
		pdf = -1.0f;
		return HitSurface(Vector3f::ZERO, Vector3f::ZERO);
	}

	Vector3f GetCenter() const { return (this->UpperRightFront + this->LowerLeftBehind) / 2.0f; }

	BBox* GetSubBox(int octant) const
	{
		Vector3f center = (this->UpperRightFront + this->LowerLeftBehind) / 2.0f;
		assert(octant >=0 && octant < 8);
		Vector3f corner;
		for (int i = 0; i < 3; i++)
			corner[i] = ((octant >> (2 - i)) & 0x1)? this->UpperRightFront[i] : this->LowerLeftBehind[i];
		return new BBox(center, corner);
	}

	bool PointInBox(const Vector3f& v) const
	{
		for (int i = 0; i < 3; i++)
			if(v[i] < this->LowerLeftBehind[i] || v[i] > this->UpperRightFront[i])
				return false;
		return true;
	}

	bool TriIntersectBox(const Vector3f& a, const Vector3f& b, const Vector3f& c) const
	{
		// An approximate intersection detection using AABB of the triangle
		bool result = true;
		for (int i = 0; i < 3; i++)
		{
			float MaxCoord = std::max(a[i], std::max(b[i], c[i])) + 1e-6;
			float MinCoord = std::min(a[i], std::min(b[i], c[i])) - 1e-6;
			result &= (MinCoord < this->UpperRightFront[i]) && (MaxCoord > this->LowerLeftBehind[i]);
		}
		return result;
	}

protected:
	Vector3f UpperRightFront;
	Vector3f LowerLeftBehind;
};

class Octree;

class Mesh : public Object3D
{

public:
	Mesh(const char *filename, Material *m);
	~Mesh();

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
	std::map<std::string, Material*> MeshMaterial;

	friend class Octree;
	Octree* tree = nullptr;
};

class Octree
{
private:
	struct OctNode
	{
		std::vector<size_t> index;
		BBox *BoundingBox = nullptr;
		bool isLeaf = false;
		OctNode *ChildNode[8];

		~OctNode()
		{
			delete this->BoundingBox;
			if (!this->isLeaf)
			{
				for (int i = 0; i < 8; i++)
					delete this->ChildNode[i];
			}
		}
	};
	const int MaxSize = 16;		// Max number of triangles in a box
	const int MaxDepth = 8;		// Max depth of the tree

	OctNode *root = nullptr;
	Mesh *mesh = nullptr;

	OctNode* Add(BBox* BoundingBox, const std::vector<size_t>& IdxList, int depth)
	{
		assert(BoundingBox != nullptr);
		if (IdxList.empty())
		{
			delete BoundingBox;
			return nullptr;
		}

		OctNode* node = new OctNode;
		node->BoundingBox = BoundingBox;
		if (IdxList.size() <= (size_t)this->MaxSize || depth >= this->MaxDepth)
		{
			node->isLeaf = true;
			node->index = IdxList;
			return node;
		}

		std::vector<size_t> list[8];
		BBox* bbox[8];
		for (int i = 0; i < 8; i++)
		{
			bbox[i] = BoundingBox->GetSubBox(i);
		}

		for (size_t idx : IdxList)
		{
			const Mesh::TriangleIndex& triIdx = this->mesh->t[idx];
			Vector3f vertices[3] = {this->mesh->v[triIdx.vIdx[0]], this->mesh->v[triIdx.vIdx[1]], this->mesh->v[triIdx.vIdx[2]]};
		//	bool flag = false;
			for (int i = 0; i < 8; i++)
			{
				if (bbox[i]->TriIntersectBox(vertices[0], vertices[1], vertices[2]))
				{
					list[i].push_back(idx);
				}
			}
		}
		
		for (int i = 0; i < 8; i++)
		{
			node->ChildNode[i] = Add(bbox[i], list[i], depth + 1);
		}
		node->isLeaf = false;
		return node;
	}

public:
	Octree() = delete;
	Octree(Mesh* m) : root(nullptr), mesh(m) {}
	~Octree () { delete this->root;}

	void Build(BBox* BoundingBox)
	{
		std::vector<size_t> IdxArray(this->mesh->t.size());
		std::iota(IdxArray.begin(), IdxArray.end(), 0);
		this->root = Add(BoundingBox, IdxArray, 1);
	}
	bool Traverse(OctNode* node, const Ray &r, Hit &h, float tmin) const;

	bool intersect(const Ray &r, Hit &h, float tmin) const
	{
		Hit htmp = h;
		if (!this->root->BoundingBox->intersect(r, htmp, tmin))
			return false;

		if (!Traverse(this->root, r, h, tmin))
			return false;
		return true;
	}

};

#endif
