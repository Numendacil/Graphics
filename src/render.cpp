#include "render.hpp"
#include "ray.hpp"
#include "light.hpp"
#include "group.hpp"
#include "utils.hpp"
#include "camera.hpp"

void PhotonMapping::BuildPM(SceneParser& scene, RandomGenerator& rng)
{
	std::vector<Photon> Photons;
	int nLights = scene.getNumLights();

	for (int PhotonIdx = 0; PhotonIdx < this->nPhoton; PhotonIdx++)
	{
		Vector3f power;

		// Sample rar from light
		int LightIdx = rng.GetUniformInt(0, nLights - 1);
		Light* light = scene.getLight(LightIdx);
		double pdf;
		Ray ray = light->SampleRay(power, pdf, rng);
		if (pdf < 0) 
			continue;
		power =  power / pdf * nLights;

		for (int DepthCount = 0; DepthCount < this->Depth; DepthCount++)
		{
			if (!CheckValid(power)) 
				break;

			Hit hit;
			bool isLight;
			int LightIdx;
			if (!scene.intersect(ray, hit, 1e-5, isLight, LightIdx)) 
				break;
			Material* material = hit.getMaterial();
			if (material->GetType() == MaterialType::DIFFUSE)
				Photons.push_back(Photon{hit.getSurface().position, -ray.getDirection(), power});
			if (DepthCount > 0)
			{
				// Use Russian Roulette
				float prob = std::max(power[0], std::max(power[1], power[2]));
				prob = (prob > 1.0f)? 1.0f : prob;
				if (rng.GetUniformReal() >= prob)
					break;
				power = power / prob;
			}
			
			// Sample new out direction
			double pdf;
			Vector3f out;
			Vector3f tangent = GetPerpendicular(hit.getSurface().normal);
			Vector3f binormal = Vector3f::cross(hit.getSurface().normal, tangent).normalized();
			Vector3f co = material->SampleOutDir(AbsToRel(tangent, binormal, hit.getSurface().normal, -ray.getDirection()), out, TransportMode::LIGHT, pdf, rng);
			out = RelToAbs(tangent, binormal, hit.getSurface().normal, out);
			ray = Ray(hit.getSurface().position, out);
			power = power * co * std::abs(Vector3f::dot(out, hit.getSurface().normal)) / pdf;
		}
	}
	logging::INFO("Number of Photons recorded: " + std::to_string(Photons.size()));
	this->GlobalPM.Clear();
	this->GlobalPM.Set(Photons);
	logging::INFO("Building kdtree");
	this->GlobalPM.Build();
}

Vector3f PhotonMapping::GetPhotonRadiance(const Vector3f& v, const Hit& hit, RandomGenerator& rng)
{
	std::vector<int> result;
	int num = this->GlobalPM.QueryNIR(hit.getSurface().position, this->SearchRadius * this->SearchRadius, result);
	Vector3f tangent = GetPerpendicular(hit.getSurface().normal);
	Vector3f binormal = Vector3f::cross(hit.getSurface().normal, tangent).normalized();

	Vector3f color = Vector3f::ZERO;
	for (auto& idx : result)
	{
		Photon ph = this->GlobalPM[idx];
		color += ph.power * hit.getMaterial()->Shade(AbsToRel(tangent, binormal, hit.getSurface().normal, -v),
								AbsToRel(tangent, binormal, hit.getSurface().normal, ph.dir),
								TransportMode::CAMERA);
	}
	return color / (M_PI * this->SearchRadius * this->SearchRadius * this->nPhoton);
}

Vector3f PhotonMapping:: GetRadiance(const Ray& r, SceneParser& scene, RandomGenerator& rng)
{
	Ray ray = r;
	Vector3f power(1, 1, 1);
	for (int depth = 0; depth < this->Depth; depth++)
	{
		Hit hit;
		bool isLight;
		int LightIdx;
		if (!scene.intersect(ray, hit, 1e-5, isLight, LightIdx))
			return scene.getBackgroundColor();
		if (isLight)
			return power * scene.getLight(LightIdx)->GetIllumin(ray.getDirection());

		Material* material = hit.getMaterial();
		HitSurface surface = hit.getSurface();

		if (material->GetType() == MaterialType::DIFFUSE)
			return power * this->GetPhotonRadiance(ray.getDirection(), hit, rng);
		else
		{
			double pdf;
			Vector3f out;
			Vector3f tangent = GetPerpendicular(surface.normal);
			Vector3f binormal = Vector3f::cross(surface.normal, tangent).normalized();
			Vector3f co = material->SampleOutDir(AbsToRel(tangent, binormal, surface.normal, -ray.getDirection()), out, TransportMode::CAMERA, pdf, rng);
			out = RelToAbs(tangent, binormal, hit.getSurface().normal, out);
			ray = Ray(surface.position, out);
			power = power * co * std::abs(Vector3f::dot(out, surface.normal)) / pdf;
		}
	}
	return Vector3f::ZERO;
}

void PhotonMapping::Render(SceneParser& scene, Image& image)
{
	float gamma = 2.0f;
	std::vector<Vector3f> img(image.Height() * image.Width());	// Temporary stored for iteration
	RandomGenerator rng;

	for (int iteration = 0; iteration < this->iter; iteration++)
	{
		logging::INFO("Iteration " + std::to_string(iteration));
		logging::INFO("Begin building PM");
		this->BuildPM(scene, rng);
		logging::INFO("Finish building PM");
		for (int i = 0; i < image.Height(); i++)
		{
			for (int j = 0; j < image.Width(); j++)
			{
				Ray camRay = scene.getCamera()->generateRay(Vector2f(i, j));
				Vector3f col = GetRadiance(camRay, scene, rng);
				if (!CheckValid(col))
					continue;
				img[j + i * image.Width()] += col; 
			}
		}
		this->SearchRadius *= std::sqrt((iteration + this->alpha) / (iteration + 1));
	}

	for (int i = 0; i < image.Height(); i++)
	{
		for (int j = 0; j < image.Width(); j++)
		{
			Vector3f col = img[j + i * image.Width()] / this->iter;
			for (int ii = 0; ii < 3; ii++)
			{
				col[ii] = (col[ii] > 1.0f)? 1.0f: col[ii];
				col[ii] = std::pow(col[ii], 1.0f / gamma);
			}
			image.SetPixel(i, j, col);
		}
	}
}