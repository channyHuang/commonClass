#ifndef FRUSTUM_CULLING
#define FRUSTUM_CULLING

#include <unordered_map>
#include <unordered_set>

#include "box.h"

class Element
{
public:
    Element() {}

    uint64_t id_ = 0;
    Box aabb_;
    void* userData_;
};

class Octant
{
public:
    Octant() {}
    Octant(const Box& _aabb) : aabb_(_aabb) {}
    Octant(const float _extent) { aabb_.vMin = Vector3(0); aabb_.vMax = Vector3(_extent); }
    ~Octant() {}

    void release() {
        if (childCount <= 0) return;
        for (int i = 0; i < 8; ++i) {
            if (children[i]) children[i]->release();
            delete  children[i];
        }
    }

    float getExtent() { return aabb_.vExtent.y; }
    void moveAABB(const float _offset) { aabb_.vMin += _offset; aabb_.vMax += _offset; }
    void moveAABB(const Vector3& _offset) { aabb_.vMin += _offset; aabb_.vMax += _offset; }

    bool addElement(Element* _element) {
        if (aabb_.getDY() < _element)
    }

    Box aabb_;
    std::unordered_set<uint64_t> elementsId_;
    std::unordered_set<Element*> elements_;
    Octant* children[8] = {nullptr};
    uint8_t childCount = 0;
};

class Octree
{
public:
    Octree() {}

    Octree(const float _minExtent) { minExtent_ = _minExtent; }

    ~Octree() {
        if (root_) {
            root_->release();
            delete root_;
        }
    }

    bool addElement(Element* _element) {
        if (_element->id_ != 0) return false;
        _element->id_ = ++elementCount_;

        if (root_ == nullptr) {
            float elementExtent = _element->aabb_.getDMax();
            float targetExtent = minExtent_;
            while (targetExtent <= elementExtent) {
                targetExtent += targetExtent;
            }
            root_ = new Octant(targetExtent);
            root_->moveAABB(_element->aabb_.getCenter() - targetExtent * 0.5f);
        }
        else while (!(_element->aabb_.inside(root_->aabb_))) {
            Octant *newRoot = new Octant(root_->getExtent() * 2.f);

            root_ = newRoot;
        }
        root_->addElement(_element);

        return true;
    }

    bool removeElement(const uint64_t _id) {
        auto itr = elements_.find(_id);
        if (itr == elements_.end()) return false;



        elementCount_--;
        return true;
    }

private:
    uint64_t elementCount_ = 0;
    Octant* root_ = nullptr;
    float minExtent_ = 1.f;
    std::unordered_map<int, Element*> elements_;
};

#endif
