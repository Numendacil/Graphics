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
{/*
	for (int argNum = 1; argNum < argc; ++argNum)
	{
		std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
	}

	if (argc != 3)
	{
		cout << "Usage: ./bin/PA1 <input scene file> <output bmp file>" << endl;
		return 1;
	}*/
	string inputFile = "test.txt";
	string outputFile = "a.bmp"; // only bmp is allowed.

	
	SceneParser sceneParser(inputFile.c_str());
	Camera *camera = sceneParser.getCamera();
	Image image(camera->getWidth(), camera->getHeight());
	PhotonMapping pm(200000, 150, 100, 16, 0.1, 0.75);
	pm.Render(sceneParser, image);

	image.SaveBMP(outputFile.c_str());
	return 0;
}
