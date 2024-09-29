#pragma once

#include <iostream>

typedef void (*CBFun_Callback)(const osg::Vec3& vPos, void* pUser);

class PickHandler : public osgGA::GUIEventHandler {
public:
	PickHandler() {}

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) {
		switch (ea.getEventType()) {
		case(osgGA::GUIEventAdapter::PUSH): {
			osgViewer::Viewer* pViewer = dynamic_cast<osgViewer::Viewer*>(&aa);
			pick(pViewer, ea);
		}
		return false;

		default:
			return false;
		}
	}

	void pick(osgViewer::Viewer* pViewer, const osgGA::GUIEventAdapter& ea)
	{
		if (!m_bCheckHit) return;
		if (!m_pFunc) return;

		osg::Group* pGroupRoot = dynamic_cast<osg::Group*>(pViewer->getSceneData());
		if (!pGroupRoot) return;

		osgUtil::LineSegmentIntersector::Intersections intersections;
		if (pViewer->computeIntersections(ea, intersections)) {
			const osgUtil::LineSegmentIntersector::Intersection& hit = *intersections.begin();

			bool bHandleMovingModels = false;
			const osg::NodePath& nodePath = hit.nodePath;
			for (osg::NodePath::const_iterator nitr = nodePath.begin();
				nitr != nodePath.end();
				++nitr) {
				const osg::Transform* pTransform = dynamic_cast<const osg::Transform*>(*nitr);
				if (pTransform)	{
					if (pTransform->getDataVariance() == osg::Object::DYNAMIC) bHandleMovingModels = true;
					break;
				}
			}

			osg::Vec3 vPos = bHandleMovingModels ? hit.getLocalIntersectPoint() : hit.getWorldIntersectPoint();
			if (!bHandleMovingModels) {
				m_pFunc(vPos, m_pUser);
			}
		}
	}

	void setCheckHit(bool bCheckHit = false) {
		m_bCheckHit = bCheckHit;
	}

	void setCallback(CBFun_Callback pFunc = nullptr, void *pUser = nullptr) {
		m_pFunc = pFunc;
		m_pUser = pUser;
	}

public:
	bool m_bCheckHit = false;

protected:
	virtual ~PickHandler() {}

	CBFun_Callback m_pFunc = nullptr;
	void* m_pUser = nullptr;
};
