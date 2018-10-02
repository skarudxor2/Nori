#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <pcg32.h>
#include <vector>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <fstream>
#include <Eigen/src/Geometry/Transform.h>

#define MAXDEPTH 16

NORI_NAMESPACE_BEGIN


class Path_mis : public Integrator
{
public:
	Path_mis(const PropertyList &props)
	{
	}
	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
	{
		Intersection its; //fisrt surface interaction

		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);

		std::vector<Mesh*> lights = scene->m_lights;

		Color3f result(0);
		Point3f x = its.p;

		int maxdepth = MAXDEPTH;


		if (its.mesh->isEmitter())
			return its.mesh->getEmitter()->getRad();

		float weight = 1.0f;

		result += recursive(scene, weight, x, sampler, its, 0, ray);
		//ray = 들어온 ray. x = 맞은 점.

		return result / (float)maxdepth;

	}

	Color3f recursive(const Scene *scene, float weight, Point3f x, Sampler *sampler, Intersection its, int depth, Ray3f ray) const
	{
		int maxdepth = MAXDEPTH;

		if (depth > maxdepth)
			return 0;
		if (!scene->rayIntersect(ray))
			return 0;
		if (its.mesh->isEmitter())
			return its.mesh->getEmitter()->getRad();

		Color3f result(0);
		std::vector<Mesh*> lights = scene->m_lights;
		int idx = (int)(sampler->next1D() * 100) % lights.size();
		const Mesh* areaLight = lights[idx];
		Point3f y;
		Normal3f n;
		float pd; //p_light
		Vector3f wi;
		const BSDF *bsdf = its.mesh->getBSDF();
		bool isDiffuse = bsdf->isDiffuse();
		const Emitter* emitter = areaLight->getEmitter();
		areaLight->sample(y, n, pd, sampler->next2D(), sampler->next1D());
		Color3f rad = emitter->getRad();

		Point3f xR, xT;
		Ray3f pathRayT, pathRayR, pathRay;

		wi = (y - x).normalized();
		BSDFQueryRecord bquery(wi);

		if (isDiffuse)
			bquery.wi = its.shFrame.toLocal(wi);
		else
			bquery.wi = -wi;

		bquery.n = its.shFrame.n.normalized();
		bquery.its = its;


		float g = G(scene, x, y, its.shFrame.n.normalized(), n.normalized());
		Color3f kr = bsdf->sample(bquery, sampler->next2D());
		Color3f kt = 1 - kr;
		Color3f fr = bsdf->eval(bquery); // = p_brdf

		float weight_term = 1.f;


		result += weight * fr* g * rad /pd ;

		Intersection _its;

		wi = ray.d.normalized();
		if (isDiffuse)
			wi = its.shFrame.toLocal(-wi);
		bquery.wi = wi;




		kr = bsdf->sample(bquery, sampler->next2D());
		kt = 1 - kr;
		fr = bsdf->eval(bquery);

		if (isDiffuse)
		{
			pathRay = Ray3f(x, its.shFrame.toWorld(bquery.wo));

			if (scene->rayIntersect(pathRay, _its))
			{
				x = _its.p;
				result += fr * recursive(scene, weight*weight_term, x, sampler, _its, depth + 1, pathRay);
			}
		}
		else
		{
			if (bquery.wt != Vector3f(0.f))
			{
				pathRayT = Ray3f(x, bquery.wt);
				if (scene->rayIntersect(pathRayT, _its))
				{
					xT = _its.p;
					result += kt * recursive(scene, weight*weight_term, xT, sampler, _its, depth + 1, pathRayT);
				}
			}
			if (bquery.wr != Vector3f(0.f))
			{
				pathRayR = Ray3f(x, bquery.wr);
				if (scene->rayIntersect(pathRayR, _its))
				{
					xR = _its.p;
					result += kr * recursive(scene, weight*weight_term, xR, sampler, _its, depth + 1, pathRayR);
				}
			}
		}
		// reflect refract weight를 어떻게 넣어야대

		return result;
	}



	float G(const Scene *scene, Point3f x, Point3f y, Normal3f n_x, Normal3f n_y) const
	{
		Vector3f x2y = (y - x).normalized();
		Vector3f y2x = (x - y).normalized();

		if (n_y.dot(y2x) <= 0)
			return 0.f;


		float numerator = abs(n_x.dot(x2y))*abs(n_y.dot(y2x));

		float dist = (x - y).norm();

		Ray3f ray(y, y2x, Epsilon, dist - Epsilon);
		bool V = !scene->rayIntersect(ray); //arealight 아닌 mesh와만 intersect check?

		return V * numerator / (x - y).squaredNorm();
	}
	//Geometric term


	std::string toString() const
	{
		return "Path_mis[]";
	}
};

NORI_REGISTER_CLASS(Path_mis, "path_mis");
NORI_NAMESPACE_END
