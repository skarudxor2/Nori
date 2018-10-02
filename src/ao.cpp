#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <pcg32.h>
#include <Eigen/src/Geometry/Transform.h>

NORI_NAMESPACE_BEGIN

class AoIntegrator : public Integrator
{
public:
	AoIntegrator(const PropertyList &props)
	{
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
	{
		/* Find the surface that is visible in the requested direction */

		Intersection its;

		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);

		//integral over the upper hemisphere centered at the point x.

		pcg32 rng;

		Vector3f normal = its.shFrame.n; // shading normal at x 
		Frame axis(normal);
		Point3f x = its.p;
		
		int n = 200;
		

		float result=0.f;
		
		for (int i = 0; i <n ; ++i)
		{
			Point3f hemi = Warp::squareToCosineHemisphere(Point2f(sampler->next2D()));


			Point3f p = axis.toWorld(hemi)+x;
			


			Vector3f w = p - x;
		//	float theta = acosf(normal.dot(w) / w.norm()); // angle between direction w and shading normal n.

			
			Ray3f _ray(x, w);

			if (!scene->rayIntersect(_ray, its))
				result += INV_PI * normal.dot(w) / w.norm();

			
			//cout << " hemi = " << hemi.toString() << endl;
			//cout << " theta = " << theta << endl;
			//cout << " cos(theta) = " << cos(theta) << endl;
		
		}
		

		return  result * M_PI * 1.5f / n;

	}


	std::string toString() const
	{
		return "AoIntegrator[]";
	}
};

NORI_REGISTER_CLASS(AoIntegrator, "ao");
NORI_NAMESPACE_END
