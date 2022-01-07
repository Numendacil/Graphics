#ifndef RENDER_H
#define RENDER_H

#include "photon_map.hpp"
#include "scene_parser.hpp"
#include "utils.hpp"
#include "image.hpp"
#include "hit.hpp"

class PhotonMapping
{
private:
	PhotonMap GlobalPM;

	int nPhoton;
	int iter;
	int Depth;
	float SearchRadius;
	float alpha;

	void BuildPM(SceneParser& scene, RandomGenerator& rng);
	Vector3f GetPhotonRadiance(const Vector3f& v, const Hit& hit, RandomGenerator& rng);
	Vector3f GetRadiance(const Ray& r, SceneParser& scene, RandomGenerator& rng);
public:
	PhotonMapping(int n, int i, int d, float r, float a) : nPhoton(n), iter(i), Depth(d), SearchRadius(r), alpha(a) {}
	void Render(SceneParser& scene, Image& image);
};
#endif