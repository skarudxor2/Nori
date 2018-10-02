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
#include <nori/common.h>
#include <nori/warp.h>
#include <fstream>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN


/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    Color3f eval(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    float pdf(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

	Vector3f reflect(const BSDFQueryRecord &bRec ) const 
	{
		return bRec.wi - 2 * bRec.wi.dot(bRec.n)*bRec.n;
	}

	 Vector3f refract(const BSDFQueryRecord &bRec, float eta) const 
	 {
		 //eta = n1/n2 = etai/etat = ext/int

		 Normal3f N = bRec.n;
		 float  cosThetaI = clamp(bRec.n.dot(bRec.wi), -1.f, 1.f);

		 if (cosThetaI < 0)
		 {
			 cosThetaI = -cosThetaI;
			
		 }
		 else
		 {
			 eta = 1 / eta;
			 N = -N;
		 }

		 float k = 1 - eta * eta*(1 - cosThetaI * cosThetaI); //c1


		 if (k < 0)
			 return 0;
		 else
			 return eta * bRec.wi + (eta*cosThetaI - sqrtf(k))*N;

	 }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const
	{

		float etaI = m_extIOR;
		float etaT = m_intIOR;
		bRec.eta = etaI / etaT;

		bRec.measure = EDiscrete;
		float cosThetaI = bRec.n.dot(bRec.wi);
	
		float kr = fresnel(cosThetaI, m_intIOR, m_extIOR); 
		


		bRec.wr = reflect(bRec);

		
		bRec.wt = refract(bRec, bRec.eta); 
		
	
		return kr;


    }

    std::string toString() const {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }

private:
    float m_intIOR, m_extIOR;

};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
