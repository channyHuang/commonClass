#pragma once

#include <iostream>

#include "osgCameraHandler.h"

CameraHandler::CameraHandler() {}

CameraHandler::CameraHandler(osgViewer::Viewer& viewer) { 
	m_pViewer = &viewer;
	m_pViewer->getCamera()->setViewMatrixAsLookAt(m_vEye, m_vCenter, m_vUp);
}

CameraHandler::~CameraHandler() {}

bool CameraHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) {
	switch (ea.getEventType()) {
	case(osgGA::GUIEventAdapter::KEYUP):
	{
		// move camera (up/down/left/right/front/back)
		m_pViewer->getCamera()->getViewMatrixAsLookAt(m_vEye, m_vCenter, m_vUp, m_fViewDistance);

		osg::Vec3 vLookDir = (m_vCenter - m_vEye);
		vLookDir.normalize();
		osg::Vec3 vLookLeft = m_vUp ^ vLookDir;
		vLookLeft.normalize();

		vLookLeft = vLookLeft * m_fStepScale;
		vLookDir = vLookDir * m_fStepScale;

		switch (ea.getKey()) {
		case osgGA::GUIEventAdapter::KEY_Q:
			m_vEye += m_vUp * m_fStepScale;
			m_vCenter += m_vUp * m_fStepScale;
			break;
		case osgGA::GUIEventAdapter::KEY_E:
			m_vEye -= m_vUp * m_fStepScale;
			m_vCenter -= m_vUp * m_fStepScale;
			break;
		case osgGA::GUIEventAdapter::KEY_W:
			m_vEye += vLookDir;
			m_vCenter += vLookDir;
			break;
		case osgGA::GUIEventAdapter::KEY_S:
			m_vEye -= vLookDir;
			m_vCenter -= vLookDir;
			break;
		case osgGA::GUIEventAdapter::KEY_A:
			m_vEye += vLookLeft;
			m_vCenter += vLookLeft;
			break;
		case osgGA::GUIEventAdapter::KEY_D:
			m_vEye -= vLookLeft;
			m_vCenter -= vLookLeft;
			break;
		default:
			return false;
		}

		m_fViewDistance = (m_vEye - m_vCenter).length();
		m_pViewer->getCamera()->setViewMatrixAsLookAt(m_vEye, m_vCenter, m_vUp);
	}
	break;
	case (osgGA::GUIEventAdapter::SCROLL):
	{
		m_pViewer->getCamera()->getViewMatrixAsLookAt(m_vEye, m_vCenter, m_vUp, m_fViewDistance);

		osg::Vec3 vLookDir = (m_vCenter - m_vEye);
		vLookDir.normalize();
		vLookDir *= m_fStepScale;
		switch (ea.getScrollingMotion())
		{
		case(osgGA::GUIEventAdapter::SCROLL_DOWN):
			m_vEye += vLookDir;
			m_vCenter += vLookDir;
			break;
		case(osgGA::GUIEventAdapter::SCROLL_UP):
			m_vEye -= vLookDir;
			m_vCenter -= vLookDir;
			break;
		default:
			return false;
		}
		m_fViewDistance = (m_vEye - m_vCenter).length();
		m_pViewer->getCamera()->setViewMatrixAsLookAt(m_vEye, m_vCenter, m_vUp);
	}
	break;
	case (osgGA::GUIEventAdapter::DRAG): {
		if (m_fPreX < 0 || m_fPreY < 0) {
			m_fPreX = ea.getX();
			m_fPreY = ea.getY();
			return false;
		}

		m_pViewer->getCamera()->getViewMatrixAsLookAt(m_vEye, m_vCenter, m_vUp, m_fViewDistance);
		osg::Vec3 vLookDir = (m_vCenter - m_vEye);
		vLookDir.normalize();

		osg::Matrix matRotUp = osg::Matrix::rotate(osg::inDegrees((ea.getX() - m_fPreX) * m_fStepScale), m_vUp);

		m_vEye = m_vCenter - matRotUp * vLookDir * m_fViewDistance;

		vLookDir = (m_vCenter - m_vEye);
		vLookDir.normalize();
		osg::Vec3 vLookLeft = m_vUp ^ (m_vCenter - m_vEye);
		vLookLeft.normalize();

		osg::Matrix matRotLeft = osg::Matrix::rotate(osg::inDegrees((ea.getY() - m_fPreY) * m_fStepScale), vLookLeft);
		m_vEye = m_vCenter - matRotLeft * vLookDir * m_fViewDistance;
		m_vUp = matRotLeft * m_vUp;
		m_vUp.normalize();
	
		m_fViewDistance = (m_vEye - m_vCenter).length();
		m_pViewer->getCamera()->setViewMatrixAsLookAt(m_vEye, m_vCenter, m_vUp);
		m_fPreX = ea.getX();
		m_fPreY = ea.getY();
	}
	break;
	case (osgGA::GUIEventAdapter::RELEASE):	{
		m_fPreX = -1;
		m_fPreY = -1;
	}
	break;
	default:
		return false;
	}

	return false;
}

void CameraHandler::reset() {
	m_vCenter = osg::Vec3(0.f, 0.f, 0.f);
	m_vEye = osg::Vec3(0.f, 10.f, 0.f);
	m_vUp = osg::Vec3(0.f, 0.f, 1.f);
	m_fViewDistance = 10.f;
	m_pViewer->getCamera()->setViewMatrixAsLookAt(m_vEye, m_vCenter, m_vUp);
}

void CameraHandler::back2WorldCenter() {
	m_pViewer->getCamera()->getViewMatrixAsLookAt(m_vEye, m_vCenter, m_vUp, m_fViewDistance);

	m_vCenter = osg::Vec3(0.f, 0.f, 0.f);
	m_fViewDistance = (m_vEye - m_vCenter).length();
	m_pViewer->getCamera()->setViewMatrixAsLookAt(m_vEye, m_vCenter, m_vUp);
}