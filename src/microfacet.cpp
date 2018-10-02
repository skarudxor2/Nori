/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
public:
	Microfacet(const PropertyList &propList) {
		/* RMS surface roughness */
		m_alpha = propList.getFloat("alpha", 0.1f);

		/* Interior IOR (default: BK7 borosilicate optical glass) */
		m_intIOR = propList.getFloat("intIOR", 1.5046f);

		/* Exterior IOR (default: air) */
		m_extIOR = propList.getFloat("extIOR", 1.000277f);

		/* Albedo of the diffuse base material (a.k.a "kd") */
		m_kd = propList.getColor("kd", Color3f(0.5f));

		/* To ensure energy conservation, we must scale the
		   specular component by 1-kd.

		   While that is not a particularly realistic model of what
		   happens in reality, this will greatly simplify the
		   implementation. Please see the course staff if you're
		   interested in implementing a more realistic version
		   of this BRDF. */
		m_ks = 1 - m_kd.maxCoeff();
	}

	float CHIPlus(const float c) const
	{
		return 0.0f < c ? 1.0f : 0.0f;
	}

	float G1(float alpha, const Vector3f &v, const Vector3f &m) const
	{
		
		if (v.dot(m) * Frame::cosTheta(v) <= 0.0f)
			return 0.0f;

		float c = (v.dot(m)) / (Frame::cosTheta(v));
		float b = 1.0f / (alpha * Frame::tanTheta(v));

		if (b < 1.6)
			return CHIPlus(c) * ((3.535f * b + 2.181f * b * b) / (1.0f + 2.276f * b + 2.577f * b * b));
		else
			return CHIPlus(c);
	}

	float G(float alpha, const Vector3f &i, const Vector3f &o, const Vector3f &m)const 
	{
		return G1(alpha, i, m) * G1(alpha, o, m);
	}

    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord &bRec) const 
	{

		/* This is a smooth BRDF -- return zero if the measure
		is wrong, or when queried for illumination on the backside */
		if (bRec.measure != ESolidAngle
			|| Frame::cosTheta(bRec.wi) <= 0
			|| Frame::cosTheta(bRec.wo) <= 0)
			return Color3f(0.0f);

		Vector3f wh = ((bRec.wi + bRec.wo)*0.5).normalized();

		float cosThetai = Frame::cosTheta(bRec.wi);
		float cosThetao = Frame::cosTheta(bRec.wo);


		//cout << "thetai = ArcCos["<< cosThetai << "];"<<endl;
		//cout << "thetao = ArcCos[" << cosThetao << "];" << endl;
		//cout << "thetawh = ArcCos[" << Frame::cosTheta(wh) << "];" << endl<<endl;

		//cout << "i dot n = " << bRec.wi.dot(bRec.n)<<endl;
		//cout << "o dot n = " << bRec.wo.dot(bRec.n)<<endl;
		//cout << "wh dot n = " << wh.dot(bRec.n)<<endl<<endl;

		//cout << "toWorld(i) dot n = " << bRec.its.shFrame.toWorld(bRec.wi).dot(bRec.n) << endl;
		//cout << "toWorld(o) dot n = " << bRec.its.shFrame.toWorld(bRec.wo).dot(bRec.n) << endl;
		//cout << "toWorld(wh) dot n = " << bRec.its.shFrame.toWorld(wh).dot(bRec.n) << endl;

		//cout << "wi = {" << bRec.wi.x() << "," << bRec.wi.y() << "," << bRec.wi.z() << "}" << endl;
		//cout << "wo = {" << bRec.wo.x() << "," << bRec.wo.y() << "," << bRec.wo.z() << "}" << endl;
		//cout << "wh = {" << wh.x() << "," << wh.y() << "," << wh.z() << "}" << endl;
		//cout << "n = {" << bRec.n.x() << "," << bRec.n.y() << "," << bRec.n.z() << "}" << endl;

		//system("Pause");

		// wi, wo, wh 다 shFrame.n에 대한 local


		
		//compute the Beckman term
		float d = Warp::squareToBeckmannPdf(wh, m_alpha);
		
		//compute the Fresnel term
		float f = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);

		//compute the geometry term
		float g = G(m_alpha, bRec.wi, bRec.wo, wh);
		//G = G1(bRec.wi, wh) * G1(bRec.wo, wh);


		

		return (m_kd * INV_PI) +(m_ks * ((d * f * g) / (4.0f * cosThetai * cosThetao * Frame::cosTheta(wh))));
		
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
	float pdf(const BSDFQueryRecord &bRec) const
	{
		if (bRec.measure != ESolidAngle
			|| Frame::cosTheta(bRec.wi) <= 0
			|| Frame::cosTheta(bRec.wo) <= 0)
			return 0.0f;

		Vector3f wh = (bRec.wi + bRec.wo).normalized();

		//compute the Beckman term devided by another cosine
		//float D = Warp::squareToBeckmannPdf(wh, m_alpha) / Frame::cosTheta(wh);
		float d = Warp::squareToBeckmannPdf(wh, m_alpha);

		float cosThetao = Frame::cosTheta(bRec.wo);

		//Jacobian of the half direction mapping
		float J = 1.0f / (4.0f * (wh.dot(bRec.wo)));

		float term1 = m_ks * d  * J;
		float term2 = (1.0f - m_ks) * cosThetao * INV_PI;

		return term1 + term2;

	

    }




	// Note: Once you have implemented the part that computes the scattered
	// direction, the last part of this function should simply return the
	// BRDF value divided by the solid angle density and multiplied by the
	// cosine factor from the reflection equation, i.e.
	// return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const 
	{
	/*	if (Frame::cosTheta(bRec.wi) <= 0)
			return Color3f(0.0f);
*/
		bRec.measure = ESolidAngle;
		/* Warp a uniformly distributed sample on [0,1]^2
		to a direction on a cosine-weighted hemisphere */
		if (_sample.x() < m_ks) 
		{
			float x = _sample.x() / m_ks;

			Vector3f n = Warp::squareToBeckmann(Point2f(x, _sample.y()), m_alpha);

			//wr = 들어오는방향 in local, n = 나가는방향 normal in local

			//Vector3f d = bRec.wi;
			//Vector3f r = d - 2.f*(d.dot(n))*n;

			bRec.wo = Warp::squareToBeckmann(Point2f(x, _sample.y()), m_alpha);
		}
		else 
		{
			
			float x = (_sample.x() - m_ks) / (1.0f - m_ks);


			bRec.wo = Warp::squareToCosineHemisphere(Point2f(x, _sample.y()));

		
		}


		if (Frame::cosTheta(bRec.wo) <= 0)
		{
			return Color3f(0.0f);
		}

		/* Relative index of refraction: no change */
		bRec.eta = 1.0f;
		float pdfVal = pdf(bRec);
		return (pdfVal != 0.0f) ? Color3f(eval(bRec) * Frame::cosTheta(bRec.wo) / pdfVal) : Color3f(0.0f);

    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    std::string toString() const {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kd.toString(),
            m_ks
        );
    }
private:
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
