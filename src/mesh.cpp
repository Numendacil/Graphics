#include "mesh.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>
#include <algorithm>

#include "plane.hpp"

Material* GenerateMaterial(const Vector3f& Ka, const Vector3f& Kd, const Vector3f& Ks, float Ns, float Ni, float d, int illum, bool hasTexture, const std::string& filename)
{
	if (illum == 0 || illum == 1)
		return hasTexture? new Lambert(Kd, filename) : new Lambert(Kd);
	else if (illum == 2)
		return hasTexture? new Phong(Kd, Ks, Ns, filename) : new Phong(Kd, Ks, Ns);
	else if (illum == 5)
		return hasTexture? new Mirror(Ks, filename) : new Mirror(Ks);
	else if (illum == 7)
		return hasTexture? new Transparent(Ks, Ni, filename) : new Transparent(Ks, Ni);
	else
		return hasTexture? new Generic(Ka, Kd, Ks, Ns, Ni, d, filename) : new Generic(Ka, Kd, Ks, Ns, Ni, d);
}

bool Octree::Traverse(Octree::OctNode* node, const Ray &r, Hit &h, float tmin) const
{
	if (node == nullptr)
		return false;
	if (node->isLeaf)
	{
		bool result = false;
		for (size_t idx : node->index)
		{
			Mesh* m = this->mesh;
			Mesh::TriangleIndex triIdx = m->t[idx];
			Triangle triangle(m->v[triIdx.vIdx[0]], m->v[triIdx.vIdx[1]], m->v[triIdx.vIdx[2]], triIdx.material);
			if (triIdx.hasNormal)
				triangle.SetNormal(m->n[triIdx.nIdx[0]], m->n[triIdx.nIdx[1]], m->n[triIdx.nIdx[2]]);
			if (triIdx.hasTexture)
				triangle.SetTexCoord(m->texcoord[triIdx.texIdx[0]], m->texcoord[triIdx.texIdx[1]], m->texcoord[triIdx.texIdx[2]]);
			result |= triangle.intersect(r, h, tmin);
		}
		return result;
		/*
		bool result = node->BoundingBox->intersect(r, h, tmin);
		h.set(h.getT(), this->mesh->material, h.getSurface());
		return result;*/
	}

	std::vector<std::pair<float, int>> tList;
	for (int octant = 0; octant < 8; octant++)
	{
		Hit hit = h;
		if (node->ChildNode[octant] != nullptr)
		{
			if (node->ChildNode[octant]->BoundingBox->intersect(r, hit, tmin))
				tList.push_back({hit.getT(), octant});
		}
	}

	std::sort(tList.begin(), tList.end());
	bool result = false;
	for (auto& p : tList)
	{
		result |= Traverse(node->ChildNode[p.second], r, h, tmin);
		if (result && node->ChildNode[p.second]->BoundingBox->PointInBox(h.getSurface().position))
			break;
	}
	return result;
}

bool Mesh::intersect(const Ray &r, Hit &h, float tmin) const
{

	// Optional: Change this brute force method into a faster one.
	return this->tree->intersect(r, h, tmin);
}

HitSurface Mesh::SamplePoint(double &pdf, RandomGenerator &rng) const
{
	int size = this->t.size();
	pdf = 1.0f / size;
	int idx = rng.GetUniformInt(0, size - 1);
	double objpdf;
	TriangleIndex triIdx = this->t[idx];
	Triangle triangle(this->v[triIdx.vIdx[0]], this->v[triIdx.vIdx[1]], this->v[triIdx.vIdx[2]], triIdx.material);
	if (triIdx.hasNormal)
		triangle.SetNormal(this->n[triIdx.nIdx[0]], this->n[triIdx.nIdx[1]], this->n[triIdx.nIdx[2]]);
	if (triIdx.hasTexture)
		triangle.SetTexCoord(this->texcoord[triIdx.texIdx[0]], this->texcoord[triIdx.texIdx[1]], this->texcoord[triIdx.texIdx[2]]);
	HitSurface s = triangle.SamplePoint(objpdf, rng);
	pdf *= objpdf;
	return s;
}

Mesh::~Mesh()
{ 
	delete this->tree; 
	for (auto& p : this->MeshMaterial) 
		delete p.second;
}

Mesh::Mesh(const char *filename, Material *material) : Object3D(material)
{

	// Optional: Use tiny obj loader to replace this simple one.
	std::ifstream f;
	f.open(filename);
	if (!f.is_open())
	{
		logging::ERROR("Cannot open " + std::string(filename));
		return;
	}

	std::string line;
	const std::string vTok("v");
	const std::string fTok("f");
	const std::string texTok("vt");
	const std::string nTok("vn");
	const std::string mtlTok("mtllib");
	const std::string useTok("usemtl");
	const char bslash = '/', space = ' ';
	std::string tok;
	Vector3f max(-INFINITY, -INFINITY, -INFINITY);
	Vector3f min(INFINITY, INFINITY, INFINITY);

	Material* curMaterial = this->material;
	logging::INFO("Begin loading " + std::string(filename)); 
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
		if (tok == mtlTok)
		{
			std::string mtlfilename;
			ss >> mtlfilename;
			parseMtl(mtlfilename);
		}
		else if (tok == useTok)
		{
			std::string name;
			ss >> name;
			if (this->MeshMaterial.count(name))
				curMaterial = this->MeshMaterial[name];
			else
				curMaterial = this->material;
		}
		else if (tok == vTok)
		{
			Vector3f vec;
			ss >> vec[0] >> vec[1] >> vec[2];
			for (int i = 0; i < 3; i++)
			{
				max[i] = std::max(max[i], vec[i]);
				min[i] = std::min(min[i], vec[i]);
			}
			this->v.push_back(vec);
		}
		else if (tok == texTok)
		{
			Vector2f texcoord;
			ss >> texcoord[0];
			ss >> texcoord[1];
			this->texcoord.push_back(texcoord);
		}
		else if (tok == nTok)
		{
			Vector3f norm;
			ss >> norm[0] >> norm[1] >> norm[2];
			this->n.push_back(norm);
		}
		else if (tok == fTok)
		{
			std::string token[3];
			TriangleIndex Idx;
			ss >> token[0] >> token[1] >> token[2];
			for (int i = 0; i < 3; i++)
			{
				std::istringstream iss(token[i]);
				std::string item;
				std::getline(iss, item, '/');	// Vertex
				Idx.vIdx[i] = std::stoi(item) - 1;
				if (std::getline(iss, item, '/'))	// Texture
				{
					if(!item.empty())
					{
						Idx.texIdx[i] = std::stoi(item) - 1;
						Idx.hasTexture = true;
					}
				}
				if (std::getline(iss, item, '/'))	// Normal
				{
					Idx.nIdx[i] = std::stoi(item) - 1;
					Idx.hasNormal = true;
				}

			}
			Idx.material = curMaterial;
			this->t.push_back(Idx);
		}
	}
	logging::INFO(std::string(filename) + " loading finished, " + std::to_string(this->v.size()) + " vertices "+ std::to_string(this->t.size()) + " triangles");
	f.close();

	BBox* BoundingBox = new BBox(max, min);
	logging::INFO("Begin building octree");
	this->tree = new Octree(this);
	this->tree->Build(BoundingBox);
/*
	for (auto& p : this->MeshMaterial)
	{
		if (p.second->HasTexture())
			p.second->SaveTexture(("tmp/" + p.first + ".bmp").c_str());
	}*/
}

void Mesh::parseMtl(const string& filename)
{
	std::ifstream f;
	f.open(filename);
	if (!f.is_open())
	{
		logging::ERROR("Cannot open " + std::string(filename));
		return;
	}

	std::string line, tok;

	bool isReading = false;
	std::string mtlName, texPath;
	Vector3f Ka, Kd, Ks;
	float Ns, Ni, d;
	int illum;
	bool hasTexture;
	logging::INFO("Begin loading " + std::string(filename)); 
	while(true)
	{
		std::getline(f, line);
		if (f.eof())
		{
			if (isReading)
				this->MeshMaterial[mtlName] = GenerateMaterial(Ka, Kd, Ks, Ns, Ni, d, illum, hasTexture, texPath);
			break;
		}
		if (line.empty())
		{
			if (isReading)
				this->MeshMaterial[mtlName] = GenerateMaterial(Ka, Kd, Ks, Ns, Ni, d, illum, hasTexture, texPath);
			isReading = false;
			continue;
		}
		if (line.at(0) == '#')
		{
			continue;
		}

		std::stringstream ss(line);
		ss >> tok;
		if (tok == "newmtl")
		{
			isReading = true;
			Ka = Vector3f(1, 1, 1);
			Kd = Vector3f(1, 1, 1);
			Ks = Vector3f(1, 1, 1);
			Ns = 0.0f;
			Ni = 1.0f;
			d = 1.0f;
			illum = -1;
			hasTexture = false;
			ss >> mtlName;
		}
		else if (tok == "Ka")
		{
			ss >> Ka[0] >> Ka[1] >> Ka[2];
		}
		else if (tok == "Kd")
		{
			ss >> Kd[0] >> Kd[1] >> Kd[2];
		}
		else if (tok == "Ks")
		{
			ss >> Ks[0] >> Ks[1] >> Ks[2];
		}
		else if (tok == "Ns")
		{
			ss >> Ns;
		}
		else if (tok == "Ni")
		{
			ss >> Ni;
		}
		else if (tok == "d")
		{
			ss >> d;
		}
		else if (tok == "illum")
		{
			ss >> illum;
		}
		else if (tok.substr(0, 3) == "map")
		{
			ss >> texPath;
			hasTexture = true;
		}
	}
	logging::INFO(std::string(filename) + " loading finished, " + std::to_string(this->MeshMaterial.size()) + " materials");
}