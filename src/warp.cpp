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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN


Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

 float intervalToTent(float sample)  //P-1(xi)
 {
	float sign;

	if (sample < 0.5f) {
		sign = 1;
		sample *= 2;
	}
	else {
		sign = -1;
		sample = 2 * (sample - 0.5f);
	}

	return sign * (1 - std::sqrt(sample));
}

 float fTent(float t)
 {
	 if (-1 <= t && t <= 1)
		 return 1 - abs(t);
	 else
		 return 0;
 }


Point2f Warp::squareToTent(const Point2f &sample) // input sample = (s,t) where (s,t) is elem of [0,1)x[0,1). and return warped 2d(or 3d) point in new domain.
{
	return Point2f(
		intervalToTent(sample.x()),
		intervalToTent(sample.y())
	);
}

float Warp::squareToTentPdf(const Point2f &p) 
{
	return fTent(p.x())*fTent(p.y());
}

//part1 


Point2f Warp::squareToUniformDisk(const Point2f &sample) 
{
	float x = sample.x();
	float y = sample.y();
	float r = sqrtf(x);
	float theta = 2.0f * M_PI*y;

	return Point2f(r*cosf(theta), r*sinf(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) 
{
	float r = sqrtf(p.x()*p.x()+p.y()*p.y());
	if (r <= 1)
		return  INV_PI;
	else
		return 0;
}


//part2

float max(float a, float b)
{
	if (a > b)
		return a;
	else return b;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) 
{
	float z = 1.f - 2.f*sample.x();
	float r = sqrtf(max(0.f, 1.f - z * z));
	float phi = 2.f*M_PI*sample.y();
	float x = r * cosf(phi);
	float y = r * sinf(phi);

	return Vector3f(x, y, z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) 
{
		return INV_FOURPI;

}



//part3


Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) 
{
	float x, y, z;

	z = sample.x();

	float r = sqrtf(max(0.f, 1.f - z * z));
	float phi = 2 * M_PI*sample.y();



	x = r * cosf(phi);
	y = r * sinf(phi);



	return Vector3f(x, y, z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v)
{
	if (Frame::cosTheta(v) >= 0)
		return INV_TWOPI;
	else return 0;

}

//part4

Point2f squareToUniformDiskConcentric(const Point2f &sample) {
	float r, theta;

	float sx = 2 * sample.x() - 1;
	float sy = 2 * sample.y() - 1;

	if (sx == 0 && sy == 0) {
		r = theta = 0;
	}
	else if (sx*sx > sy*sy) {
		r = sx;
		theta = (M_PI / 4.0f) * (sy / sx);
	}
	else {
		r = sy;
		theta = (M_PI / 2.0f) - (sx / sy) * (M_PI / 4.0f);
	}


	return Point2f(r * cosf(theta), r * sinf(theta));
}




Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) 
{
	Point2f p = squareToUniformDiskConcentric(sample);
	float z = sqrtf(max(0.f,1.0f - p.x()*p.x() - p.y()*p.y()));



	return Vector3f(p.x(), p.y(), z);

}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) 
{
	float f;
	if (Frame::cosTheta(v) >= 0)
		f = Frame::cosTheta(v);
	else f = 0;
	return INV_PI *f ;
}

//part5





Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) 
{
	float phi = 2 * M_PI * sample.x();
	float theta = atanf(sqrtf(-1.f*alpha*alpha*logf(  -sample.y()+ 1 )));
	




	return Vector3f( sinf(theta)*cosf(phi), sinf(theta)*sinf(phi), cosf(theta));
}


float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) 
{
	float theta = acos(m.z());

	float result = INV_TWOPI * 2 * expf(-1.f*tanf(theta)*tanf(theta)	/	(alpha*alpha)	) /( alpha * alpha *m.z()*m.z()*m.z());
	
	if (m.z() <= 0)
		result = 0;


	return result;
}

NORI_NAMESPACE_END
