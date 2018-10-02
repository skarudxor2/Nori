#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator 
{
private:
	Point3f position;
	Color3f energy;

public:
	SimpleIntegrator(const PropertyList &props) 
	{
		position=props.getPoint("position");
		energy = props.getColor("energy");
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
	{
		/* Find the surface that is visible in the requested direction */

		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);  

		float theta;
		Vector3f x = its.p;
		Vector3f direction = position-x; //from x(point being rendered) to position -> p-x
		Vector3f normal = its.shFrame.n;

		theta = acosf(normal.dot(direction)/direction.norm());

		Color3f result;
		if (cos(theta)>0)
			result = energy * INV_TWOPI*INV_TWOPI* cos(theta) / (x - position).dot(x - position);
		else
			result = 0;

		//visibility check - > x에서 p까지 ray쏴서 intersect하는거 있으면 x?

		Ray3f _ray(its.p,direction);
		Intersection _its;

		if (scene->rayIntersect(_ray, _its))
			result = 0;
		

		
		return result;

		
	}

	std::string toString() const 
	{
		return "SimpleIntegrator[]";
	}
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END