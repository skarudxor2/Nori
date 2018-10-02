#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <pcg32.h>
#include <vector>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <fstream>
#include <Eigen/src/Geometry/Transform.h>



NORI_NAMESPACE_BEGIN


class WhittedIntegrator : public Integrator
{
public:
	WhittedIntegrator(const PropertyList &props)
	{
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
	{
		/* Find the surface that is visible in the requested direction */


		Intersection its; //fisrt surface interaction

		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);



		std::vector<Mesh*> lights = scene->m_lights;


		const BSDF *bsdf = its.mesh->getBSDF(); // bsdf of mesh of intersection of primary ray
		bool isDiffuse = bsdf->isDiffuse();
		Color3f result(0);
		Point3f x = its.p;

		if (isDiffuse)
		{
			const Emitter* emitter;

			for (auto areaLight : lights)
			{
				Point3f y; // sample point of area light
				Normal3f n; //interpolated surface normal at p 
				float pd; // probability density of the sample

				areaLight->sample(y, n, pd, sampler->next2D(), sampler->next1D());
				emitter = areaLight->getEmitter();

				Color3f rad = emitter->getRad(); //L_e?

				if (its.mesh->isEmitter())
					return rad;

				Vector3f wi = (y - x).normalized();
				BSDFQueryRecord bquery(its.shFrame.toLocal(wi)); //incidSent direction = wi = x -> y
				bquery.n = its.shFrame.n.normalized();
				bquery.its = its;

				bsdf->sample(bquery, sampler->next2D());
				Color3f fr = bsdf->eval(bquery);


				result += fr * G(scene, x, y, its.shFrame.n.normalized(), n.normalized())*rad / pd;

			}


		}

		else
		{

			int depth = ray.depth;
			Vector3f wi = ray.d.normalized();
			BSDFQueryRecord bquery(wi); // toLocal?



			bquery.n = its.shFrame.n.normalized();
			bquery.its = its;

			Color3f kr = bsdf->sample(bquery, sampler->next2D()); //value of fresnel eq = kr
			Color3f kt = 1 - kr;
			//mirror는 return 1
			

			float xi = sampler->next1D();

			if (depth <= 15)
			{

				if (bquery.wt != Vector3f(0.f, 0.f, 0.f))
					result += kt * Li(scene, sampler, Ray3f(x, bquery.wt, depth + 1));
				if (bquery.wr != Vector3f(0.f, 0.f, 0.f))
					result += kr * Li(scene, sampler, Ray3f(x, bquery.wr, depth + 1));
			}
			else
				result += 0.f;


		}


		return result;
	}
	
	float G(const Scene *scene, Point3f x, Point3f y,Normal3f n_x, Normal3f n_y) const 
	{
		Vector3f x2y = (y - x).normalized();
		Vector3f y2x = (x - y).normalized();
		if (n_y.dot(y2x) <= 0)
			return 0.f;

		float numerator = abs(n_x.dot(x2y))*abs(n_y.dot(y2x));

		float dist = (x - y).norm();

		Ray3f ray(y,y2x,Epsilon,dist-Epsilon);
		bool V = !scene->rayIntersect(ray); //arealight 아닌 mesh와만 intersect check?
		
		return V * numerator / (x - y).squaredNorm();
	}
	//Geometric term


	std::string toString() const
	{
		return "WhittedIntegrator[]";
	}
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END
