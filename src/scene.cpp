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

#include <nori/scene.h>
#include <nori/bitmap.h>
#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/camera.h>
#include <nori/emitter.h>
#include <nori/mesh.h>
#include <ctime>
NORI_NAMESPACE_BEGIN

Scene::Scene(const PropertyList &) {
    m_accel = new Accel();
}

Scene::~Scene() {
    delete m_accel;
    delete m_sampler;
    delete m_camera;
    delete m_integrator;

}

//implementaion of preorder traversal to check construction

void Scene::activate() 
{
	/*********************************************   below contents are inserted one      *******************************************************/

	m_accel->build();

	//MatrixXf m_N(3,0);//vertex Normals
	//MatrixXf m_V(3,0);//vertex Positions
	//MatrixXu m_F(3,0); //getIndices
	//for (auto i : m_lights)
	//{
	//	if (i == m_lights[0])
	//	{
	//		m_N.resize(m_N.rows(), m_N.cols() + i->getVertexNormals().cols());
	//		m_N << i->getVertexNormals();

	//		m_V.resize(m_V.rows(), m_V.cols() + i->getVertexPositions().cols());
	//		m_V << i->getVertexPositions();
	
	//		m_F.resize(m_F.rows(), m_F.cols() + i->getIndices().cols());
	//		m_F << i->getIndices();
	//	}
	//	else
	//	{
	//		m_N.resize(m_N.rows(), m_N.cols() + i->getVertexNormals().cols());
	//		m_N << m_N,i->getVertexNormals();

	//		m_V.resize(m_V.rows(), m_V.cols() + i->getVertexPositions().cols());
	//		m_V << m_V,i->getVertexPositions();

	//		m_F.resize(m_F.rows(), m_F.cols() + i->getIndices().cols());
	//		m_F << m_F,i->getIndices();
	//	}
	//}
	//cout << "size of m_V = "<<m_V.cols() << endl;
	//cout << "size of m_F = " << m_F.cols() << endl;
	//cout << "size of m_N = " << m_N.cols() << endl;
	//merged = new Mesh(m_N, m_V, m_F);



	/*********************************************   above contents are inserted one      *******************************************************/

    if (!m_integrator)
        throw NoriException("No integrator was specified!");
    if (!m_camera)
        throw NoriException("No camera was specified!");
    
    if (!m_sampler) {
        /* Create a default (independent) sampler */
        m_sampler = static_cast<Sampler*>(
            NoriObjectFactory::createInstance("independent", PropertyList()));
    }

    cout << endl;
    cout << "Configuration: " << toString() << endl;
    cout << endl;
}

void Scene::addChild(NoriObject *obj) {
	switch (obj->getClassType())
	{
	case EMesh:
	{
		cout << "case EMesh" << endl;
		Mesh *mesh = static_cast<Mesh *>(obj);
		m_accel->addMesh(mesh);
		m_meshes.push_back(mesh);
		if (mesh->isEmitter())
		{
			m_lights.push_back(mesh);
			cout << "m_lights.push back" << endl;
		}
	}
	break;

	case EEmitter:
	{
		cout << "case EEmitter" << endl;
		Emitter *emitter = static_cast<Emitter *>(obj);
		m_emitters.push_back(emitter);

	}
	break;

	case ESampler:
		if (m_sampler)
			throw NoriException("There can only be one sampler per scene!");
		m_sampler = static_cast<Sampler *>(obj);
		break;

	case ECamera:
		if (m_camera)
			throw NoriException("There can only be one camera per scene!");
		m_camera = static_cast<Camera *>(obj);
		break;

	case EIntegrator:
		if (m_integrator)
			throw NoriException("There can only be one integrator per scene!");
		m_integrator = static_cast<Integrator *>(obj);
		break;

	default:
		throw NoriException("Scene::addChild(<%s>) is not supported!",
			classTypeName(obj->getClassType()));
	
    }
}

std::string Scene::toString() const {
    std::string meshes;
    for (size_t i=0; i<m_meshes.size(); ++i) {
        meshes += std::string("  ") + indent(m_meshes[i]->toString(), 2);
        if (i + 1 < m_meshes.size())
            meshes += ",";
        meshes += "\n";
    }

    return tfm::format(
        "Scene[\n"
        "  integrator = %s,\n"
        "  sampler = %s\n"
        "  camera = %s,\n"
        "  meshes = {\n"
        "  %s  }\n"
        "]",
        indent(m_integrator->toString()),
        indent(m_sampler->toString()),
        indent(m_camera->toString()),
        indent(meshes, 2)
    );
}

NORI_REGISTER_CLASS(Scene, "scene");
NORI_NAMESPACE_END
