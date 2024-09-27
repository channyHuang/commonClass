#pragma once

#include <iostream>

#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>

class CameraHandler : public osgGA::GUIEventHandler {
public:
	CameraHandler();
	CameraHandler(osgViewer::Viewer& viewer);

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void reset();
	void back2WorldCenter();

public:
	bool m_bBack2WorldCenter = false;
	int m_nAxis = 0;
	float m_fStepScale = .5f;

protected:
	virtual ~CameraHandler();

protected:
	float m_fTheta = 0, m_fPhi = 0;
	float m_fPreX = -1, m_fPreY = -1;
	float m_fViewDistance = 10.f;
	osg::Vec3 m_vEye = osg::Vec3(0.f, 10.f, 0.f), m_vUp = osg::Vec3(0.f, 1.f, 0.f), m_vCenter = osg::Vec3(0.f, 0.f, 0.f);
	osg::ref_ptr<osgViewer::Viewer> m_pViewer;
	osg::Matrix m_matAxisRot = osg::Matrix::identity();

};