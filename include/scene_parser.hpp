#ifndef SCENE_PARSER_H
#define SCENE_PARSER_H

#include <cassert>
#include <vecmath.h>
#include "material.hpp"
#include "group.hpp"
#include "camera.hpp"
#include "light.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include "plane.hpp"
#include "sphere.hpp"
#include "transform.hpp"
#include "triangle.hpp"
#include "rectangle.hpp"
#include "light.hpp"
#include "mesh.hpp"

#define MAX_PARSER_TOKEN_LENGTH 1024

class SceneParser
{
public:
	SceneParser() = delete;
	SceneParser(const char *filename);

	~SceneParser();

	Camera *getCamera() const
	{
		return camera;
	}

	Vector3f getBackgroundColor() const
	{
		return background_color;
	}

	int getNumLights() const
	{
		return num_lights;
	}

	Light *getLight(int i) const
	{
		assert(i >= 0 && i < num_lights);
		return lights[i];
	}

	int getNumMaterials() const
	{
		return num_materials;
	}

	Material *getMaterial(int i) const
	{
		assert(i >= 0 && i < num_materials);
		return materials[i];
	}

	Group *getGroup() const
	{
		return group;
	}

	bool intersect(const Ray &r, Hit &h, float tmin, bool& isLight, int& LightIdx) const
	{
		bool ObjIntersect = this->group->intersect(r, h, tmin);
		isLight = false;
		for (int i = 0; i < this->num_lights; i++)
		{
			isLight |= this->lights[i]->intersect(r, h, tmin);
			if (isLight)
				LightIdx = i;
		}
		return isLight | ObjIntersect;
	}

private:
	void parseFile();

	void parsePerspectiveCamera();
	void parseLensCamera();

	void parseBackground();

	void parseLights();
	Light *parsePointLight();
	Light *parseAreaLight();

	void parseMaterials();
	Lambert *parseLambertMaterial();
	Phong *parsePhongMaterial();
	Mirror *parseMirrorMaterial();
	Transparent *parseTransparentMaterial();
	Material *parseGenericMaterial();

	Object3D *parseObject(char token[MAX_PARSER_TOKEN_LENGTH]);
	Group *parseGroup();
	Sphere *parseSphere();
	Plane *parsePlane();
	Rectangle * parseRectangle();
	Triangle *parseTriangle();
	Mesh *parseTriangleMesh();
	Transform *parseTransform();

	int getToken(char token[MAX_PARSER_TOKEN_LENGTH]);

	Vector3f readVector3f();

	float readFloat();
	int readInt();

	FILE *file;
	Camera *camera;
	Vector3f background_color;
	int num_lights;
	Light **lights;
	int num_materials;
	Material **materials;
	Material *current_material;
	Group *group;
};

#endif // SCENE_PARSER_H
