#pragma once

#include <osg/Referenced>
#include <osg/ref_ptr>
#include <osg/PolygonMode>
#include <osg/ShapeDrawable>
#include <osg/Depth>
#include <osg/LightModel>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/TexMat>
#include <osg/TexGen>
#include <osg/TexEnv>
#include <osg/Texture2D>
#include <osg/TextureCubeMap>
#include <osg/TexEnvCombine>
#include <osg/TextureRectangle>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>
#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osgUtil/TangentSpaceGenerator>
#include <osg/PositionAttitudeTransform>
#include <osg/Math>
#include <osg/Matrix>
#include <osg/AutoTransform>

#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowSettings>
#include <osgShadow/SoftShadowMap>

#include <osgUtil/PerlinNoise>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgText/Text>

#include "commonFunc.h"

#include <unordered_map>

struct Pipeline
{
	int textureSize;
	osg::ref_ptr<osg::Group> graph;
	osg::Texture* pass1Shadows;
	osg::Texture* pass2Colors;
	osg::Texture* pass2Normals;
	osg::Texture* pass2Positions;
	osg::Texture* pass3Final;
};

struct GeometryFinder : public osg::NodeVisitor
{
    osg::ref_ptr<osg::Geometry> _geom;
    GeometryFinder() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}
    void apply(osg::Geode& geode)
    {
        if (_geom.valid())
            return;
        for (unsigned int i = 0; i < geode.getNumDrawables(); i++)
        {
            osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
            if (geom) {
                _geom = geom;
                return;
            }
        }
    }
};


enum MatrixType {
	MVP,
	ModelView,
	Project,
	Normal
};

class MatrixCallback : public osg::UniformCallback {
public:
	MatrixCallback(osg::Camera* camera, MatrixType _type = MVP) : m_camera(camera), type(_type) {}
	MatrixCallback(osgViewer::Viewer* viewer, MatrixType _type = MVP) : pviewer(viewer), type(_type) {}

	virtual void operator ()(osg::Uniform* uniform, osg::NodeVisitor* nv) {
		if (pviewer != nullptr) m_camera = pviewer->getCamera();
		osg::Matrixd mat;
		switch (type) {
		case MVP:
		{
			osg::Matrixd modelView = m_camera->getViewMatrix();
			osg::Matrixd projMat = m_camera->getProjectionMatrix();
			mat = modelView * projMat;
		}
		break;
		case ModelView:
			mat = m_camera->getViewMatrix();
			break;
		case Project:
			mat = m_camera->getProjectionMatrix();
			break;
		case Normal:
		{
			osg::Matrixd modelView = m_camera->getViewMatrix();
			mat = osg::Matrixd::inverse(modelView);
		}
		break;
		default:
			mat = osg::Matrixd::identity();
		}

		uniform->set(mat);
	}

private:
	MatrixType type = MVP;
	osg::Camera* m_camera;
	osgViewer::Viewer* pviewer = nullptr;
};

class MVPCallback : public osg::UniformCallback {
public:
	MVPCallback(osg::Camera* camera) : m_pCamera(camera) {}

	virtual void operator ()(osg::Uniform* uniform, osg::NodeVisitor* nv) {
		osg::Matrixd modelView = m_pCamera->getViewMatrix();
		osg::Matrixd projMat = m_pCamera->getProjectionMatrix();
		osg::Matrixd mvp = modelView * projMat;
		uniform->set(mvp);
	}

private:
	osg::Camera* m_pCamera;
};
class ViewCallback : public osg::UniformCallback {
public:
	ViewCallback(osg::Camera* camera) : m_pCamera(camera) {}

	virtual void operator ()(osg::Uniform* uniform, osg::NodeVisitor* nv) {
		osg::Matrixd modelView = m_pCamera->getViewMatrix();
		osg::Matrixd viewInv = osg::Matrixd::inverse(modelView);
		uniform->set(viewInv);
	}

private:
	osg::Camera* m_pCamera;
};
class LightPosCallback : public osg::UniformCallback {
public:
	LightPosCallback(osg::Camera* camera) : m_pCamera(camera) {}

	virtual void operator ()(osg::Uniform* uniform, osg::NodeVisitor* nv) {
		osg::Vec3f eye, center, up;
		m_pCamera->getViewMatrixAsLookAt(eye, center, up);
		uniform->set(eye);
	}

private:
	osg::Camera* m_pCamera;
};
class NormalCallback : public osg::UniformCallback {
public:
	NormalCallback(osg::Camera* camera) : m_camera(camera) {}

	virtual void operator ()(osg::Uniform* uniform, osg::NodeVisitor* nv) {
		osg::Matrixd modelView = m_camera->getViewMatrix();
		osg::Matrixd viewInv = osg::Matrixd::inverse(modelView);
		osg::Matrixd viewInvTr;
		viewInvTr.transpose3x3(viewInv);
		osg::Matrix3 res;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				res(i, j) = viewInvTr(i, j);
			}
		}

		uniform->set(res);
	}

private:
	osg::Camera* m_camera;
};

enum ShowType {
	SHOW_FRONT_AND_BACK,
	SHOW_WIREFRAME,
	SHOW_FRONT,
	SHOW_BACK
};
extern osg::Node* createOsgGlslNode(osg::ref_ptr<osg::Geometry> pGeom, osgViewer::Viewer* pViewer, std::string sTextrueName);
extern bool loadGeomUsingTinyobj(const char* filename, const char* basepath, osgViewer::Viewer* pViewer, bool bUseGlsl = false);
extern osg::ref_ptr<osg::Geometry> createGlslGeomUsingTinyobj(const char* filename, const char* basepath, osgViewer::Viewer* pViewer);
extern osg::ref_ptr<osg::Geometry> createGeomUsingTinyobj(const char* filename, const char* basepath);
extern osg::ref_ptr<osg::Geometry> loadOsgNode(const std::string& sFileName);

extern osg::Node* createText(const osg::Vec3& position, float characterSize, const std::string& message, float minScale = 0.0, float maxScale = FLT_MAX);
extern osg::ref_ptr<osg::LineWidth> getNewLineWidth(float width = 2.f);
extern osg::ref_ptr<osg::Geode> createAxis(float len = 10.0);
extern osg::Node* createSkyBox();
extern osg::Group* createSunLight();
extern osg::ref_ptr<osg::LightSource> createLight(const osg::Vec3& pos, int num, const osg::BoundingSphere& bs, osg::ref_ptr<osg::Group> root);
extern osg::TextureCubeMap* readCubeMap();
extern osg::ref_ptr<osg::Program> loadShaderPrograms(const std::string& name,
													 const std::string& vshaderName, const std::string& fshaderName);
extern osg::ref_ptr<osg::Material> createMaterial();
extern osg::Texture2D* createTexture(const std::string& fileName);
extern GLuint createTexture(const unsigned char* data, int width, int height);
extern osg::Geometry* createPlanet(double radius, const osg::Vec4& color = osg::Vec4(1.f, 1.f, 1.f, 1.f), const osg::Vec3& pos = osg::Vec3(0.f, 0.f, 0.f));
extern void setTexture(const std::string& name, osg::StateSet* stateset);
// type 0: both sides; 1: wireFrame; 2: single side
extern void setWireFrame(osg::StateSet* stateset, ShowType type = SHOW_FRONT_AND_BACK);
extern void clearSingleGroup(osg::ref_ptr<osg::Group>& node);
extern void setTexCombine(osg::StateSet* stateset);
extern osg::TextureRectangle* createFloatTextureRectangle(int textureSize);
extern std::vector<std::string> splitString(std::string& str, char c = '\t');

extern float calcObjProjectArea(std::string sMeshName, std::string sPath = "");
