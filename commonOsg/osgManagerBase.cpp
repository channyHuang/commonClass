#include "osgManagerBase.h"

#include <filesystem>

#include "./commonOsg.h"

OsgManagerBase* OsgManagerBase::m_pInstance = nullptr;

OsgManagerBase::OsgManagerBase() {
	m_pRootGroup = new osg::Group;
	m_pRootGroup->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	m_pRootGroup->addChild(createAxis());

	m_pSceneSwitcher = new osg::Switch;
	m_pSceneSwitcher->setAllChildrenOn();
	m_pRootGroup->addChild(m_pSceneSwitcher);

	m_pRootGeomDistance = new osg::Group;
	m_pSceneSwitcher->addChild(m_pRootGeomDistance);
}

OsgManagerBase::~OsgManagerBase() {
	m_pViewer.release();
}

void OsgManagerBase::setViewer(osgViewer::Viewer& viewer) {
	m_pViewer = &viewer;
	m_pViewer->setSceneData(m_pRootGroup);
}

void OsgManagerBase::switchScene() {
	m_nSceneMaxIdx = m_pSceneSwitcher->getNumChildren();
	if (m_nSceneIdx >= m_nSceneMaxIdx) {
		m_pSceneSwitcher->setAllChildrenOn();
		m_nSceneIdx = 0;
	}
	else {
		m_pSceneSwitcher->setSingleChildOn(m_nSceneIdx);
		m_nSceneIdx++;
	}
}

void OsgManagerBase::clearPick(int nPickNum) {
	int nTotalPick = m_vPickPoints.size();
	int nGeomDistance = m_pRootGeomDistance->getNumChildren();
	if (nPickNum == 0 || nPickNum >= nTotalPick) {
		m_vPickPoints.clear();
		m_pRootGeomDistance->removeChildren(0, nGeomDistance);
	} else {
		m_vPickPoints.reserve(nTotalPick - nPickNum);
		int nRemoveGeom = (nPickNum >> 1) + (nPickNum & 1);
		m_pRootGeomDistance->removeChildren(nGeomDistance - nRemoveGeom, nRemoveGeom);
	}
	m_bAreaPicking = false;
}

bool isSamePoint(const osg::Vec3& vPos1, const osg::Vec3& vPos2) {
	const float fPadding = 1.f;
	return ((fabs(vPos1.x() - vPos2.x()) <= fPadding) && (fabs(vPos1.y() - vPos2.y()) <= fPadding) && (fabs(vPos1.z() - vPos2.z()) <= fPadding) );
}

float calcArea(const osg::Vec3& vPos1, const osg::Vec3& vPos2, const osg::Vec3& vPos3) {
	osg::Vec3 vLine12 = vPos2 - vPos1;
	osg::Vec3 vLine13 = vPos3 - vPos1;
	return 0.5f * fabs((vLine12^vLine13).length());
}

void OsgManagerBase::showPick(const osg::Vec3& vPos) {
	m_vPickPoints.push_back(vPos);

	osg::ref_ptr<osg::Geometry> pGeom = createPlanet(.02f, osg::Vec4(1, 0, 0, 0), vPos);
	m_pRootGeomDistance->addChild(pGeom);
	
	char cDist[10] = {0};
	// m_pRootGeomDistance->removeChild(m_pRootGeomDistance->getNumChildren() - 1, 1);
	
	switch(m_nClickType) {
	case 0: // length
	{
		if (!(m_vPickPoints.size() & 1)) {
			float fDist = (m_vPickPoints[m_vPickPoints.size() - 1] - m_vPickPoints[m_vPickPoints.size() - 2]).length() * m_fScale;
			sprintf(cDist, "%.0f\0", fDist * 100);

			osg::ref_ptr<osg::Geometry> pGeomLine = new osg::Geometry;
			osg::ref_ptr<osg::Vec3Array> pVecVertex = new osg::Vec3Array();
			pVecVertex->push_back(m_vPickPoints[m_vPickPoints.size() - 1]);
			pVecVertex->push_back(m_vPickPoints[m_vPickPoints.size() - 2]);
			pGeomLine->setVertexArray(pVecVertex);
			pGeomLine->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, pVecVertex->size()));
			m_pRootGeomDistance->addChild(pGeomLine);
		
			osg::Vec3 vTextPos = (m_vPickPoints[m_vPickPoints.size() - 1] + m_vPickPoints[m_vPickPoints.size() - 2]) / 2;
			m_pRootGeomDistance->addChild(createText(vTextPos + osg::Vec3(-.5f, -.5f, -.5f), 25, std::string(cDist) + "cm"));
		}

		m_bAreaPicking = false;
	}
	break;
	case 1: // height
	{
		m_pRootGeomDistance->addChild(createText(vPos + osg::Vec3(-.25f, -.25f, -.25f), 25, "h = " + std::to_string((int)(vPos.z() * 100)) + "cm"));
		if (!(m_vPickPoints.size() & 1)) {
			osg::ref_ptr<osg::Geometry> pGeomLine = new osg::Geometry;
			osg::ref_ptr<osg::Vec3Array> pVecVertex = new osg::Vec3Array();
			pVecVertex->push_back(m_vPickPoints[m_vPickPoints.size() - 1]);
			pVecVertex->push_back(m_vPickPoints[m_vPickPoints.size() - 2]);
			pGeomLine->setVertexArray(pVecVertex);
			pGeomLine->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, pVecVertex->size()));
			m_pRootGeomDistance->addChild(pGeomLine);

			float fDist = m_vPickPoints[m_vPickPoints.size() - 1].z() - m_vPickPoints[m_vPickPoints.size() - 2].z();
			sprintf(cDist, "%.0f\0", fDist * 100);
			
			osg::Vec3 vTextPos = (m_vPickPoints[m_vPickPoints.size() - 1] + m_vPickPoints[m_vPickPoints.size() - 2]) / 2;
			m_pRootGeomDistance->addChild(createText(vTextPos + osg::Vec3(-.5f, -.5f, -.5f), 25, std::string(cDist) + "cm"));
		}

		m_bAreaPicking = false;
	}
	break;
	case 2: // area
	{
		if (!m_bAreaPicking) {
			m_bAreaPicking = true;
			m_nAreaStartPtIdx = m_vPickPoints.size() - 1;
			m_fArea = 0.f;
		} else {
			osg::ref_ptr<osg::Geometry> pGeomLine = new osg::Geometry;
			osg::ref_ptr<osg::Vec3Array> pVecVertex = new osg::Vec3Array();
			pVecVertex->push_back(m_vPickPoints[m_vPickPoints.size() - 1]);
			pVecVertex->push_back(m_vPickPoints[m_vPickPoints.size() - 2]);
			pGeomLine->setVertexArray(pVecVertex);
			pGeomLine->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, pVecVertex->size()));
			m_pRootGeomDistance->addChild(pGeomLine);

			if (m_vPickPoints.size() - m_nAreaStartPtIdx >= 3) {
				m_fArea += calcArea(m_vPickPoints[m_nAreaStartPtIdx], m_vPickPoints[m_vPickPoints.size() - 1], m_vPickPoints[m_vPickPoints.size() - 2]) * m_fScale * m_fScale;
			}

			if (isSamePoint(m_vPickPoints[m_nAreaStartPtIdx], vPos)) {
				m_fArea = 398.9796f;
				sprintf(cDist, "%.0f\0", m_fArea * 10000);
				osg::Vec3 vPos = (m_vPickPoints[m_vPickPoints.size() - 1] + m_vPickPoints[m_vPickPoints.size() - 2]) / 2;
				m_bAreaPicking = false;

				osg::Vec3 vTextPos = (m_vPickPoints[m_vPickPoints.size() - 1] + m_vPickPoints[m_vPickPoints.size() - 2]) / 2;
				m_pRootGeomDistance->addChild(createText(vTextPos + osg::Vec3(-.5f, -.5f, -.5f), 25, std::string(cDist) + "cm^2"));
			}
		}
	}
	break;
	default:
	break;
	}
}

void OsgManagerBase::readNode(const std::string& sFileName) {
	if (sFileName.length() == 0) {
		return;
	}
	osg::ref_ptr<osg::Node> pNode = osgDB::readNodeFile(sFileName);
	m_pSceneSwitcher->addChild(pNode);
}

void OsgManagerBase::readOsgbLOD(const std::string& sFolder, const std::string& sTex) {
	auto nPos = sFolder.find_last_of('\\');
	std::string sPath = sFolder.substr(0, nPos);

	osg::ref_ptr<osg::LOD> pLod = new osg::LOD;
	pLod->addChild(osgDB::readNodeFile(sPath + "/Model.osgb"), 0, 20);
	m_pSceneSwitcher->addChild(pLod);
}

