#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include "scene_parser.hpp"
#include "camera.hpp"
#include "light.hpp"
#include "material.hpp"
#include "object3d.hpp"
#include "group.hpp"
#include "mesh.hpp"
#include "sphere.hpp"
#include "plane.hpp"
#include "triangle.hpp"
#include "transform.hpp"

#define DegreesToRadians(x) ((M_PI * x) / 180.0f)

SceneParser::SceneParser(const char *filename)
{

	// initialize some reasonable default values
	group = nullptr;
	camera = nullptr;
	background_color = Vector3f(0.5, 0.5, 0.5);
	num_lights = 0;
	lights = nullptr;
	num_materials = 0;
	materials = nullptr;
	current_material = nullptr;

	// parse the file
	assert(filename != nullptr);
	const char *ext = &filename[strlen(filename) - 4];

	if (strcmp(ext, ".txt") != 0)
	{
		printf("wrong file name extension\n");
		exit(0);
	}
	file = fopen(filename, "r");

	if (file == nullptr)
	{
		printf("cannot open scene file\n");
		exit(0);
	}
	parseFile();
	fclose(file);
	file = nullptr;

	if (num_lights == 0)
	{
		printf("WARNING:    No lights specified\n");
	}
}

SceneParser::~SceneParser()
{

	delete group;
	delete camera;

	int i;
	for (i = 0; i < num_materials; i++)
	{
		delete materials[i];
	}
	delete[] materials;
	for (i = 0; i < num_lights; i++)
	{
		delete lights[i];
	}
	delete[] lights;
}

// ====================================================================
// ====================================================================

void SceneParser::parseFile()
{
	//
	// at the top level, the scene can have a camera,
	// background color and a group of objects
	// (we add lights and other things in future assignments)
	//
	char token[MAX_PARSER_TOKEN_LENGTH];
	while (getToken(token))
	{
		if (!strcmp(token, "PerspectiveCamera"))
		{
			parsePerspectiveCamera();
		}
		else if (!strcmp(token, "LensCamera"))
		{
			parseLensCamera();
		}
		else if (!strcmp(token, "Background"))
		{
			parseBackground();
		}
		else if (!strcmp(token, "Lights"))
		{
			parseLights();
		}
		else if (!strcmp(token, "Materials"))
		{
			parseMaterials();
		}
		else if (!strcmp(token, "Group"))
		{
			group = parseGroup();
		}
		else
		{
			printf("Unknown token in parseFile: '%s'\n", token);
			exit(0);
		}
	}
}

// ====================================================================
// ====================================================================

void SceneParser::parsePerspectiveCamera()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	// read in the camera parameters
	getToken(token);
	assert(!strcmp(token, "{"));
	getToken(token);
	assert(!strcmp(token, "center"));
	Vector3f center = readVector3f();
	getToken(token);
	assert(!strcmp(token, "direction"));
	Vector3f direction = readVector3f();
	getToken(token);
	assert(!strcmp(token, "up"));
	Vector3f up = readVector3f();
	getToken(token);
	assert(!strcmp(token, "angle"));
	float angle_degrees = readFloat();
	float angle_radians = DegreesToRadians(angle_degrees);
	getToken(token);
	assert(!strcmp(token, "width"));
	int width = readInt();
	getToken(token);
	assert(!strcmp(token, "height"));
	int height = readInt();
	getToken(token);
	assert(!strcmp(token, "gamma"));
	int gamma = readFloat();
	getToken(token);
	assert(!strcmp(token, "}"));
	camera = new PerspectiveCamera(center, direction, up, width, height, gamma, angle_radians);
}

void SceneParser::parseLensCamera()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	// read in the camera parameters
	getToken(token);
	assert(!strcmp(token, "{"));
	getToken(token);
	assert(!strcmp(token, "center"));
	Vector3f center = readVector3f();
	getToken(token);
	assert(!strcmp(token, "direction"));
	Vector3f direction = readVector3f();
	getToken(token);
	assert(!strcmp(token, "up"));
	Vector3f up = readVector3f();
	getToken(token);
	assert(!strcmp(token, "angle"));
	float angle_degrees = readFloat();
	float angle_radians = DegreesToRadians(angle_degrees);
	getToken(token);
	assert(!strcmp(token, "width"));
	int width = readInt();
	getToken(token);
	assert(!strcmp(token, "height"));
	int height = readInt();
	getToken(token);
	assert(!strcmp(token, "gamma"));
	float gamma = readFloat();
	getToken(token);
	assert(!strcmp(token, "aperture"));
	float aperture = readFloat();
	getToken(token);
	assert(!strcmp(token, "focal"));
	float focal = readFloat();
	getToken(token);
	assert(!strcmp(token, "}"));
	camera = new LensCamera(center, direction, up, width, height, gamma, angle_radians, aperture, focal);
}

void SceneParser::parseBackground()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	// read in the background color
	getToken(token);
	assert(!strcmp(token, "{"));
	while (true)
	{
		getToken(token);
		if (!strcmp(token, "}"))
		{
			break;
		}
		else if (!strcmp(token, "color"))
		{
			background_color = readVector3f();
		}
		else if (!strcmp(token, "ambient"))
		{
			ambient_color = readVector3f();
		}
		else
		{
			printf("Unknown token in parseBackground: '%s'\n", token);
			assert(0);
		}
	}
}

// ====================================================================
// ====================================================================

void SceneParser::parseLights()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	getToken(token);
	assert(!strcmp(token, "{"));
	// read in the number of objects
	getToken(token);
	assert(!strcmp(token, "numLights"));
	num_lights = readInt();
	lights = new Light *[num_lights];
	// read in the objects
	int count = 0;
	while (num_lights > count)
	{
		getToken(token);
		if (strcmp(token, "AreaLight") == 0)
		{
			lights[count] = parseAreaLight();
		}
		else if (strcmp(token, "PointLight") == 0)
		{
			lights[count] = parsePointLight();
		}
		else if (strcmp(token, "DirectedPointLight") == 0)
		{
			lights[count] = parseDirectedPointLight();
		}
		else
		{
			printf("Unknown token in parseLight: '%s'\n", token);
			exit(0);
		}
		count++;
	}
	getToken(token);
	assert(!strcmp(token, "}"));
}

AreaLight *SceneParser::parseAreaLight()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	getToken(token);
	assert(!strcmp(token, "{"));
	Object3D* object = parseGroup();
	getToken(token);
	assert(!strcmp(token, "power"));
	Vector3f power = readVector3f();
	getToken(token);
	assert(!strcmp(token, "}"));
	return new AreaLight(object, power);
}

PointLight *SceneParser::parsePointLight()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	getToken(token);
	assert(!strcmp(token, "{"));
	getToken(token);
	assert(!strcmp(token, "position"));
	Vector3f position = readVector3f();
	getToken(token);
	assert(!strcmp(token, "power"));
	Vector3f power = readVector3f();
	getToken(token);
	assert(!strcmp(token, "}"));
	return new PointLight(position, power);
}

DirectedPointLight *SceneParser::parseDirectedPointLight()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	getToken(token);
	assert(!strcmp(token, "{"));
	getToken(token);
	assert(!strcmp(token, "position"));
	Vector3f position = readVector3f();
	getToken(token);
	assert(!strcmp(token, "direction"));
	Vector3f dir = readVector3f();
	getToken(token);
	assert(!strcmp(token, "angle"));
	float angle_degrees = readFloat();
	float angle_radians = DegreesToRadians(angle_degrees);
	getToken(token);
	assert(!strcmp(token, "power"));
	Vector3f power = readVector3f();
	getToken(token);
	assert(!strcmp(token, "}"));
	return new DirectedPointLight(position, dir, angle_radians,power);
}

// ====================================================================
// ====================================================================

void SceneParser::parseMaterials()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	getToken(token);
	assert(!strcmp(token, "{"));
	// read in the number of objects
	getToken(token);
	assert(!strcmp(token, "numMaterials"));
	num_materials = readInt();
	materials = new Material *[num_materials];
	// read in the objects
	int count = 0;
	while (num_materials > count)
	{
		getToken(token);
		if (!strcmp(token, "GenericMaterial"))
		{
			materials[count] = parseGenericMaterial();
		}
		else if (!strcmp(token, "PhongMaterial"))
		{
			materials[count] = parsePhongMaterial();
		}
		else if (!strcmp(token, "LambertMaterial"))
		{
			materials[count] = parseLambertMaterial();
		}
		else if (!strcmp(token, "MirrorMaterial"))
		{
			materials[count] = parseMirrorMaterial();
		}
		else if (!strcmp(token, "TransparentMaterial"))
		{
			materials[count] = parseTransparentMaterial();
		}
		else
		{
			printf("Unknown token in parseMaterial: '%s'\n", token);
			exit(0);
		}
		count++;
	}
	getToken(token);
	assert(!strcmp(token, "}"));
}


Material *SceneParser::parseGenericMaterial()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	char filename[MAX_PARSER_TOKEN_LENGTH];
	bool hasTexture = false;
	Vector3f Ka(1, 1, 1), Kd(1, 1, 1), Ks(1, 1, 1);
	float Ns = 0.0f, Ni = 1.0f, d = 1.0f;
	int illum = -1; 
	getToken(token);
	assert(!strcmp(token, "{"));
	while (true)
	{
		getToken(token);
		if (strcmp(token, "Ka") == 0)
		{
			Ka = readVector3f();
		}
		else if (strcmp(token, "Kd") == 0)
		{
			Kd = readVector3f();
		}
		else if (strcmp(token, "Ks") == 0)
		{
			Ks = readVector3f();
		}
		else if (strcmp(token, "Ns") == 0)
		{
			Ns = readFloat();
		}
		else if (strcmp(token, "Ni") == 0)
		{
			Ni = readFloat();
		}
		else if (strcmp(token, "d") == 0)
		{
			d = readFloat();
		}
		else if (strcmp(token, "illum") == 0)
		{
			illum = readInt();
		}
		else if (strcmp(token, "texture") == 0 || strcmp(token, "map_Kd") == 0)
		{
			getToken(filename);
			hasTexture = true;
		}
		else
		{
			assert(!strcmp(token, "}"));
			break;
		}
	}
	
	if (illum == 0 || illum == 1)
		return hasTexture? new Lambert(Kd, filename) : new Lambert(Kd);
	else if (illum == 2)
		return hasTexture? new Phong(Kd, Ks, Ns, filename) : new Phong(Kd, Ks, Ns);
	else if (illum == 5)
		return hasTexture? new Mirror(Ks, filename) : new Mirror(Ks);
	else if (illum == 7)
		return hasTexture?new Transparent(Ks, Ni, filename) : new Transparent(Ks, Ni);
	else
		return hasTexture? new Generic(Ka, Kd, Ks, Ns, Ni, d, filename) : new Generic(Ka, Kd, Ks, Ns, Ni, d);
}


Lambert *SceneParser::parseLambertMaterial()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	char filename[MAX_PARSER_TOKEN_LENGTH];
	bool hasTexture = false;
	Vector3f color(1, 1, 1);
	getToken(token);
	assert(!strcmp(token, "{"));
	while (true)
	{
		getToken(token);
		if (strcmp(token, "color") == 0)
		{
			color = readVector3f();
		}
		else if (strcmp(token, "texture") == 0)
		{
			getToken(filename);
			hasTexture = true;
		}
		else
		{
			assert(!strcmp(token, "}"));
			break;
		}
	}
	Lambert *answer;
	if (hasTexture)
		answer = new Lambert(color, filename);
	else
		answer = new Lambert(color);
	return answer;
}

Phong *SceneParser::parsePhongMaterial()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	char filename[MAX_PARSER_TOKEN_LENGTH];
	bool hasTexture = false;
	Vector3f diffuseColor(1, 1, 1), specularColor(0, 0, 0);
	float shininess = 0;
	getToken(token);
	assert(!strcmp(token, "{"));
	while (true)
	{
		getToken(token);
		if (strcmp(token, "diffuseColor") == 0)
		{
			diffuseColor = readVector3f();
		}
		else if (strcmp(token, "specularColor") == 0)
		{
			specularColor = readVector3f();
		}
		else if (strcmp(token, "shininess") == 0)
		{
			shininess = readFloat();
		}
		else if (strcmp(token, "texture") == 0)
		{
			getToken(filename);
			hasTexture = true;
		}
		else
		{
			assert(!strcmp(token, "}"));
			break;
		}
	}
	Phong *answer;
	if (hasTexture)
		answer = new Phong(diffuseColor, specularColor, shininess, filename);
	else
		answer = new Phong(diffuseColor, specularColor, shininess);
	return answer;
}

Mirror *SceneParser::parseMirrorMaterial()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	char filename[MAX_PARSER_TOKEN_LENGTH];
	bool hasTexture = false;
	Vector3f color(1, 1, 1);
	getToken(token);
	assert(!strcmp(token, "{"));
	while (true)
	{
		getToken(token);
		if (strcmp(token, "color") == 0)
		{
			color = readVector3f();
		}
		else if (strcmp(token, "texture") == 0)
		{
			getToken(filename);
			hasTexture = true;
		}
		else
		{
			assert(!strcmp(token, "}"));
			break;
		}
	}
	Mirror *answer;
	if (hasTexture) 
		answer = new Mirror(color, filename);
	else
		answer = new Mirror(color);
	return answer;
}

Transparent *SceneParser::parseTransparentMaterial()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	char filename[MAX_PARSER_TOKEN_LENGTH];
	bool hasTexture = false;
	Vector3f color(1, 1, 1);
	float ior = 1.0f;
	getToken(token);
	assert(!strcmp(token, "{"));
	while (true)
	{
		getToken(token);
		if (strcmp(token, "color") == 0)
		{
			color = readVector3f();
		}
		else if (strcmp(token, "index") == 0)
		{
			ior = readFloat();
		}
		else if (strcmp(token, "texture") == 0)
		{
			getToken(filename);
			hasTexture = true;
		}
		else
		{
			assert(!strcmp(token, "}"));
			break;
		}
	}
	Transparent *answer;
	if (hasTexture) 
		answer = new Transparent(color, ior, filename);
	else
		answer = new Transparent(color, ior);
	return answer;
}

// ====================================================================
// ====================================================================

Object3D *SceneParser::parseObject(char token[MAX_PARSER_TOKEN_LENGTH])
{
	Object3D *answer = nullptr;
	if (!strcmp(token, "Group"))
	{
		answer = (Object3D *)parseGroup();
	}
	else if (!strcmp(token, "Sphere"))
	{
		answer = (Object3D *)parseSphere();
	}
	else if (!strcmp(token, "Plane"))
	{
		answer = (Object3D *)parsePlane();
	}
	else if (!strcmp(token, "Rectangle"))
	{
		answer = (Object3D *)parseRectangle();
	}
	else if (!strcmp(token, "Triangle"))
	{
		answer = (Object3D *)parseTriangle();
	}
	else if (!strcmp(token, "TriangleMesh"))
	{
		answer = (Object3D *)parseTriangleMesh();
	}
	else if (!strcmp(token, "Transform"))
	{
		answer = (Object3D *)parseTransform();
	}
	else
	{
		printf("Unknown token in parseObject: '%s'\n", token);
		exit(0);
	}
	return answer;
}

// ====================================================================
// ====================================================================

Group *SceneParser::parseGroup()
{
	//
	// each group starts with an integer that specifies
	// the number of objects in the group
	//
	// the material index sets the material of all objects which follow,
	// until the next material index (scoping for the materials is very
	// simple, and essentially ignores any tree hierarchy)
	//
	char token[MAX_PARSER_TOKEN_LENGTH];
	getToken(token);
	assert(!strcmp(token, "{"));

	// read in the number of objects
	getToken(token);
	assert(!strcmp(token, "numObjects"));
	int num_objects = readInt();

	auto *answer = new Group(num_objects);

	// read in the objects
	int count = 0;
	while (num_objects > count)
	{
		getToken(token);
		if (!strcmp(token, "MaterialIndex"))
		{
			// change the current material
			int index = readInt();
			assert(index >= 0 && index <= getNumMaterials());
			current_material = getMaterial(index);
		}
		else
		{
			Object3D *object = parseObject(token);
			assert(object != nullptr);
			answer->addObject(count, object);

			count++;
		}
	}
	getToken(token);
	assert(!strcmp(token, "}"));

	// return the group
	return answer;
}

// ====================================================================
// ====================================================================

Sphere *SceneParser::parseSphere()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	getToken(token);
	assert(!strcmp(token, "{"));
	getToken(token);
	assert(!strcmp(token, "center"));
	Vector3f center = readVector3f();
	getToken(token);
	assert(!strcmp(token, "radius"));
	float radius = readFloat();
	getToken(token);
	assert(!strcmp(token, "}"));
	assert(current_material != nullptr);
	return new Sphere(center, radius, current_material);
}

Plane *SceneParser::parsePlane()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	bool hasTexture = false;
	Vector3f e1, e2, origin;
	getToken(token);
	assert(!strcmp(token, "{"));
	getToken(token);
	assert(!strcmp(token, "normal"));
	Vector3f normal = readVector3f();
	getToken(token);
	assert(!strcmp(token, "offset"));
	float offset = readFloat();
	getToken(token);
	if (!strcmp(token, "e1"))
	{
		hasTexture = true;
		e1 = readVector3f();
		getToken(token);
		assert(!strcmp(token, "e2"));
		e2 = readVector3f();
		getToken(token);
		assert(!strcmp(token, "origin"));
		origin = readVector3f();
		getToken(token);
	}
	assert(!strcmp(token, "}"));
	assert(current_material != nullptr);
	Plane* p = new Plane(normal, offset, current_material);
	if (hasTexture)
		p->SetTexCoord(e1, e2, origin);
	return p;
}

Rectangle *SceneParser::parseRectangle()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	getToken(token);
	assert(!strcmp(token, "{"));
	getToken(token);
	assert(!strcmp(token, "a"));
	Vector3f a = readVector3f();
	getToken(token);
	assert(!strcmp(token, "b"));
	Vector3f b = readVector3f();
	getToken(token);
	assert(!strcmp(token, "}"));
	assert(current_material != nullptr);
	return new Rectangle(a, b, current_material);
}

Triangle *SceneParser::parseTriangle()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	getToken(token);
	assert(!strcmp(token, "{"));
	getToken(token);
	assert(!strcmp(token, "vertex0"));
	Vector3f v0 = readVector3f();
	getToken(token);
	assert(!strcmp(token, "vertex1"));
	Vector3f v1 = readVector3f();
	getToken(token);
	assert(!strcmp(token, "vertex2"));
	Vector3f v2 = readVector3f();
	getToken(token);
	assert(!strcmp(token, "}"));
	assert(current_material != nullptr);
	return new Triangle(v0, v1, v2, current_material);
}

Mesh *SceneParser::parseTriangleMesh()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	char filename[MAX_PARSER_TOKEN_LENGTH];
	// get the filename
	getToken(token);
	assert(!strcmp(token, "{"));
	getToken(token);
	assert(!strcmp(token, "obj_file"));
	getToken(filename);
	getToken(token);
	assert(!strcmp(token, "}"));
	const char *ext = &filename[strlen(filename) - 4];
	assert(!strcmp(ext, ".obj"));
	Mesh *answer = new Mesh(filename, current_material);

	return answer;
}

Transform *SceneParser::parseTransform()
{
	char token[MAX_PARSER_TOKEN_LENGTH];
	Matrix4f matrix = Matrix4f::identity();
	Object3D *object = nullptr;
	getToken(token);
	assert(!strcmp(token, "{"));
	// read in transformations:
	// apply to the LEFT side of the current matrix (so the first
	// transform in the list is the last applied to the object)
	getToken(token);

	while (true)
	{
		if (!strcmp(token, "Scale"))
		{
			Vector3f s = readVector3f();
			matrix = matrix * Matrix4f::scaling(s[0], s[1], s[2]);
		}
		else if (!strcmp(token, "UniformScale"))
		{
			float s = readFloat();
			matrix = matrix * Matrix4f::uniformScaling(s);
		}
		else if (!strcmp(token, "Translate"))
		{
			matrix = matrix * Matrix4f::translation(readVector3f());
		}
		else if (!strcmp(token, "XRotate"))
		{
			matrix = matrix * Matrix4f::rotateX(DegreesToRadians(readFloat()));
		}
		else if (!strcmp(token, "YRotate"))
		{
			matrix = matrix * Matrix4f::rotateY(DegreesToRadians(readFloat()));
		}
		else if (!strcmp(token, "ZRotate"))
		{
			matrix = matrix * Matrix4f::rotateZ(DegreesToRadians(readFloat()));
		}
		else if (!strcmp(token, "Rotate"))
		{
			getToken(token);
			assert(!strcmp(token, "{"));
			Vector3f axis = readVector3f();
			float degrees = readFloat();
			float radians = DegreesToRadians(degrees);
			matrix = matrix * Matrix4f::rotation(axis, radians);
			getToken(token);
			assert(!strcmp(token, "}"));
		}
		else if (!strcmp(token, "Matrix4f"))
		{
			Matrix4f matrix2 = Matrix4f::identity();
			getToken(token);
			assert(!strcmp(token, "{"));
			for (int j = 0; j < 4; j++)
			{
				for (int i = 0; i < 4; i++)
				{
					float v = readFloat();
					matrix2(i, j) = v;
				}
			}
			getToken(token);
			assert(!strcmp(token, "}"));
			matrix = matrix2 * matrix;
		}
		else
		{
			// otherwise this must be an object,
			// and there are no more transformations
			object = parseObject(token);
			break;
		}
		getToken(token);
	}

	assert(object != nullptr);
	getToken(token);
	assert(!strcmp(token, "}"));
	return new Transform(matrix, object);
}

// ====================================================================
// ====================================================================

int SceneParser::getToken(char token[MAX_PARSER_TOKEN_LENGTH])
{
	// for simplicity, tokens must be separated by whitespace
	assert(file != nullptr);
	int success = fscanf(file, "%s ", token);
	if (success == EOF)
	{
		token[0] = '\0';
		return 0;
	}
	return 1;
}

Vector3f SceneParser::readVector3f()
{
	float x, y, z;
	int count = fscanf(file, "%f %f %f", &x, &y, &z);
	if (count != 3)
	{
		printf("Error trying to read 3 floats to make a Vector3f\n");
		assert(0);
	}
	return Vector3f(x, y, z);
}

float SceneParser::readFloat()
{
	float answer;
	int count = fscanf(file, "%f", &answer);
	if (count != 1)
	{
		printf("Error trying to read 1 float\n");
		assert(0);
	}
	return answer;
}

int SceneParser::readInt()
{
	int answer;
	int count = fscanf(file, "%d", &answer);
	if (count != 1)
	{
		printf("Error trying to read 1 int\n");
		assert(0);
	}
	return answer;
}
