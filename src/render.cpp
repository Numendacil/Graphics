#include "render.hpp"
#include "ray.hpp"
#include "light.hpp"
#include "group.hpp"
#include "utils.hpp"
#include "camera.hpp"
#include <omp.h>

void PhotonMapping::BuildPM(SceneParser& scene, std::vector<RandomGenerator>& rng_list)
{
	std::vector<Photon> Photons;
	int nLights = scene.getNumLights();

	#pragma omp parallel for schedule(dynamic, 100)
	for (int PhotonIdx = 0; PhotonIdx < this->nPhoton; PhotonIdx++)
	{
		Vector3f power;
		RandomGenerator& rng = rng_list[omp_get_thread_num()];

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
			if (DepthCount > 0)
			{
				// Use Russian Roulette
				float prob = std::max(power[0], std::max(power[1], power[2]));
				prob = (prob > 1.0f)? 1.0f : prob;
				if (rng.GetUniformReal() >= prob)
					break;
				power = power / prob;
			}

			Hit hit;
			bool isLight;
			int LightIdx;
			if (!scene.intersect(ray, hit, 1e-5, isLight, LightIdx)) 
				break;
			Material* material = hit.getMaterial();
			const HitSurface& surface = hit.getSurface();
			Vector3f in = -ray.getDirection().normalized();

			// Sample new out direction
			double pdf;
			RefType type;
			Vector3f out;
			Vector3f tangent = GetPerpendicular(surface.normal);
			Vector3f binormal = Vector3f::cross(surface.normal, tangent).normalized();
			Vector3f co = material->SampleOutDir(AbsToRel(tangent, binormal, surface.normal, in), out, TransportMode::LIGHT, pdf, type, rng);
			if (type == RefType::DIFFUSE)
			{
				#pragma omp critical
				{
					Photons.push_back(Photon{surface.position, in, power});
				}
			}
			if (surface.HasTexture && material->HasTexture())
			{
				co = co * material->GetTexture(surface.texcoord);
			}
			out = RelToAbs(tangent, binormal, surface.normal, out);
			ray = Ray(surface.position, out);
			power = power * co / pdf  * std::abs(Vector3f::dot(out, surface.geonormal)) * std::abs(Vector3f::dot(in, surface.normal)) / std::abs(Vector3f::dot(in, surface.geonormal));
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
	const HitSurface& surface = hit.getSurface();
	int num = this->GlobalPM.QueryNIR(surface.position, this->SearchRadius * this->SearchRadius, result);
	Vector3f tangent = GetPerpendicular(surface.normal);
	Vector3f binormal = Vector3f::cross(surface.normal, tangent).normalized();

	Vector3f color = Vector3f::ZERO;
	for (auto& idx : result)
	{
		Photon ph = this->GlobalPM[idx];
		color += ph.power * hit.getMaterial()->Shade(AbsToRel(tangent, binormal, surface.normal, -v),
								AbsToRel(tangent, binormal, surface.normal, ph.dir),
								TransportMode::CAMERA);
	};
	if (surface.HasTexture && hit.getMaterial()->HasTexture())
		color = color * hit.getMaterial()->GetTexture(surface.texcoord);
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
		Vector3f dir = ray.getDirection().normalized();

		Material* material = hit.getMaterial();
		HitSurface surface = hit.getSurface();

		double pdf;
		RefType type;
		Vector3f out;
		Vector3f tangent = GetPerpendicular(surface.normal);
		Vector3f binormal = Vector3f::cross(surface.normal, tangent).normalized();
		Vector3f co = material->SampleOutDir(AbsToRel(tangent, binormal, surface.normal, -dir), out, TransportMode::CAMERA, pdf, type, rng);
		if (type == RefType::DIFFUSE)
		{
			if (isLight)
				power = power * scene.getLight(LightIdx)->GetIllumin(dir);
			return power * this->GetPhotonRadiance(dir, hit, rng);
		}
		out = RelToAbs(tangent, binormal, hit.getSurface().normal, out);
		ray = Ray(surface.position, out);
		power = power * co * std::abs(Vector3f::dot(out, surface.normal)) / pdf;
	}
	return power;
}

void PhotonMapping::Render(SceneParser& scene, Image& image)
{
	std::vector<Vector3f> img(image.Height() * image.Width());	// Temporary stored for iteration
	std::vector<RandomGenerator> rng_list(omp_get_max_threads());
	for (size_t i = 0; i < rng_list.size(); i++)
	{
		rng_list[i].SetSeed(rng_list[i].GetSeed() + rng_list[i].GetUniformInt(0, rng_list.size()) + i * rng_list.size());
	}

	for (int iteration = 0; iteration < this->iter; iteration++)
	{
		logging::INFO("Iteration " + std::to_string(iteration));
		logging::INFO("Begin building PM");
		this->BuildPM(scene, rng_list);
		logging::INFO("Finish building PM");
		int count = 0;

		Image image_tmp(image.Width(), image.Height()); 
		#pragma omp parallel for collapse(2) schedule(dynamic, 5)
		for (int i = 0; i < image.Width(); i++)
		{
			for (int j = 0; j < image.Height(); j++)
			{
				RandomGenerator& rng = rng_list[omp_get_thread_num()];
				Vector3f col = Vector3f::ZERO;
				for (int k = 0; k < this->nRays; k++)
				{
					Ray camRay = scene.getCamera()->SampleRay(i, j, rng);
					Vector3f co = GetRadiance(camRay, scene, rng);
					if (!CheckValid(co))
						continue;
					col += co;
				}
				img[j + i * image.Height()] += col / this->nRays;
				Vector3f col_tmp = img[j + i * image.Height()] / (iteration + 1);
				for (int ii = 0; ii < 3; ii++)
				{
					col_tmp[ii] = (col_tmp[ii] > 1.0f) ? 1.0f : col_tmp[ii];
					col_tmp[ii] = std::pow(col_tmp[ii], 1.0f / scene.getCamera()->getGamma());
				}
				image_tmp.SetPixel(i, j, col_tmp);
				#pragma omp critical
				{
					count++;
					if (!(count % 10000))
						logging::INFO(std::to_string(count) + "/" + std::to_string(image.Width() * image.Height()) + " pixels finished");
				}
			}
		}
		image_tmp.SaveBMP(("tmp/" + std::to_string(iteration) + ".bmp").c_str());
		this->SearchRadius *= std::sqrt((iteration + this->alpha) / (iteration + 1));
	}

	for (int i = 0; i < image.Width(); i++)
	{
		for (int j = 0; j < image.Height(); j++)
		{
			Vector3f col = img[j + i * image.Height()] / this->iter;
			for (int ii = 0; ii < 3; ii++)
			{
				col[ii] = (col[ii] > 1.0f)? 1.0f: col[ii];
				col[ii] = std::pow(col[ii], 1.0f / scene.getCamera()->getGamma());
			}
			image.SetPixel(i, j, col);
		}
	}
}