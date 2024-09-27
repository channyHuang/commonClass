#pragma once

#include <iostream>

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
		if (!bCheckHit) return;

		osg::Group* pGroupRoot = dynamic_cast<osg::Group*>(pViewer->getSceneData());
		if (!pGroupRoot) return;

		osgUtil::LineSegmentIntersector::Intersections intersections;
		if (pViewer->computeIntersections(ea, intersections)) {
			const osgUtil::LineSegmentIntersector::Intersection& hit = *intersections.begin();

			bool bHandleMovingModels = false;
			const osg::NodePath& nodePath = hit.nodePath;
			for (osg::NodePath::const_iterator nitr = nodePath.begin();
				nitr != nodePath.end();
				++nitr)
			{
				const osg::Transform* transform = dynamic_cast<const osg::Transform*>(*nitr);
				if (transform)
				{
					if (transform->getDataVariance() == osg::Object::DYNAMIC) bHandleMovingModels = true;
				}
			}

			osg::Vec3 position = bHandleMovingModels ? hit.getLocalIntersectPoint() : hit.getWorldIntersectPoint();
			if (!bHandleMovingModels) {
				//TerrainModification::getInstance()->modify((TerrainModifyType)(OsgManager::getInstance()->modifyType), Vector3(position.x(), position.y(), position.z()));

				
			}
		}
	}

	void setValid(bool _valid) {
		bCheckHit = _valid;
	}

public:
	bool bCheckHit = false;

protected:
	virtual ~PickHandler() {}
};
