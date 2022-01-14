#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>

#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"
#include "render.hpp"

#include <string>

using namespace std;

int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		cout << "Usage: ./bin/PA1 <input scene file> <output bmp file>" << endl;
		return 1;
	}
	string inputFile = argv[1];
	string outputFile = argv[2] + std::string(".bmp"); // only bmp is allowed.

	
	SceneParser sceneParser(inputFile.c_str());
	Camera *camera = sceneParser.getCamera();
	Image image(camera->getWidth(), camera->getHeight());
	PhotonMapping pm(200000, 200, 100, 16, 0.2, 0.75);
	pm.Render(sceneParser, image);

	image.SaveBMP(outputFile.c_str());
	return 0;
}
