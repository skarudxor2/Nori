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

#pragma once

#include <nori/vector.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Simple n-dimensional ray segment data structure
 * 
 * Along with the ray origin and direction, this data structure additionally
 * stores a ray segment [mint, maxt] (whose entries may include positive/negative
 * infinity), as well as the componentwise reciprocals of the ray direction.
 * That is just done for convenience, as these values are frequently required.
 *
 * \remark Important: be careful when changing the ray direction. You must
 * call \ref update() to compute the componentwise reciprocals as well, or Nori's
 * ray-triangle intersection code will go haywire.
 */
template <typename _PointType, typename _VectorType> struct TRay {
    typedef _PointType                  PointType;
    typedef _VectorType                 VectorType;
    typedef typename PointType::Scalar  Scalar;

    PointType o;     ///< Ray origin
    VectorType d;    ///< Ray direction
    VectorType dRcp; ///< Componentwise reciprocals of the ray direction
    Scalar mint;     ///< Minimum position on the ray segment
    Scalar maxt;     ///< Maximum position on the ray segment
	int depth;


    /// Construct a new ray
    TRay() : mint(Epsilon), 
		maxt(std::numeric_limits<Scalar>::infinity()) {
		depth = 0;
	}
    
    /// Construct a new ray
    TRay(const PointType &o, const VectorType &d) : o(o), d(d), 
            mint(Epsilon), maxt(std::numeric_limits<Scalar>::infinity()) {
		depth = 0;
        update();
    }

	TRay(const PointType &o, const VectorType &d, int depth) : o(o), d(d),
		mint(Epsilon), maxt(std::numeric_limits<Scalar>::infinity()),depth(depth) {
		update();
	}
    /// Construct a new ray
    TRay(const PointType &o, const VectorType &d, 
        Scalar mint, Scalar maxt) : o(o), d(d), mint(mint), maxt(maxt) {
		depth = 0;
        update();
    }

    /// Copy constructor
    TRay(const TRay &ray) 
     : o(ray.o), d(ray.d), dRcp(ray.dRcp),
       mint(ray.mint), maxt(ray.maxt), depth(ray.depth) { }

    /// Copy a ray, but change the covered segment of the copy
    TRay(const TRay &ray, Scalar mint, Scalar maxt) 
     : o(ray.o), d(ray.d), dRcp(ray.dRcp), mint(mint), maxt(maxt), depth(ray.depth) { }

    /// Update the reciprocal ray directions after changing 'd'
    void update() {
        dRcp = d.cwiseInverse();
    }

    /// Return the position of a point along the ray
    PointType operator() (Scalar t) const { return o + t * d; }

    /// Return a ray that points into the opposite direction
    TRay reverse() const {
        TRay result;
        result.o = o; result.d = -d; result.dRcp = -dRcp;
        result.mint = mint; result.maxt = maxt;
        return result;
    }

    /// Return a human-readable string summary of this ray
    std::string toString() const {
        return tfm::format(
                "Ray[\n"
                "  o = %s,\n"
                "  d = %s,\n"
                "  mint = %f,\n"
                "  maxt = %f\n"
                "]", o.toString(), d.toString(), mint, maxt);
    }
};

NORI_NAMESPACE_END
