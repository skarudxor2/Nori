#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <pcg32.h>

/**
* \brief Superclass of all emitters
*
* some mechanism for sample generation,
* evaluation of probabilities, and for
* returning the emitted radiance is needed.
*/

NORI_NAMESPACE_BEGIN
class AreaLight : public Emitter
{
private:
	Color3f radiance;

public:
	Point2f sample2D() const
	{
		pcg32 rng((uint64_t)this);

		return Point2f(rng.nextFloat(), rng.nextFloat());
	}

	AreaLight(const PropertyList &props)
	{
		radiance = props.getColor("radiance");
	}


	Color3f getRad() const
	{
		return radiance;
	}

	std::string toString() const
	{
		return "AreaLight[] rad = " + radiance.toString();
	}
};

NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END
