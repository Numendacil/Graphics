#ifndef GROUP_H
#define GROUP_H

#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>

// TODO: Implement Group - add data structure to store a list of Object*
class Group : public Object3D
{

public:
	Group()
	{
	}

	explicit Group(int num_objects)
	{
		this->ObjList.reserve((size_t)num_objects);
	}

	~Group() override
	{
		for (Object3D *item : this->ObjList)
			delete item;
	}

	bool intersect(const Ray &r, Hit &h, float tmin) const override
	{
		bool result = false;
		for (Object3D *item : this->ObjList)
			result |= item->intersect(r, h, tmin);
		return result;
	}

	HitSurface SamplePoint(double &pdf, RandomGenerator &rng) const override
	{
		int size = this->ObjList.size();
		pdf = 1.0f / size;
		int idx = rng.GetUniformInt(0, size - 1);
		double objpdf;
		HitSurface s = this->ObjList[idx]->SamplePoint(objpdf, rng);
		pdf *= objpdf;
		return s;
	}

	void addObject(int index, Object3D *obj)
	{
		this->ObjList.push_back(obj);
	}

	int getGroupSize()
	{
		return this->ObjList.size();
	}

private:
	std::vector<Object3D *> ObjList;
};

#endif
