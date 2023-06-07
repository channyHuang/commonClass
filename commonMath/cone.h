#ifndef CONE_H
#define CONE_H

#include "vector3.h"
#include "boxi.h"
#include "math_funcs.h"

class Cone
{
public:
    Cone(Vector3 vcenter, float fRadius, float fHeight)
        : m_vCenter(vcenter), m_fRadius(fRadius), m_fHeight(fHeight) {};

    float getSdf(Vector3 vPos);
    Boxi getBox();
    Vector3 getCenter() { return m_vCenter; }
    Vector3 getTop() { return m_vCenter + Vector3(0, m_fHeight, 0); }

private:
    Vector3 m_vCenter;
    float m_fRadius;
    float m_fHeight;
};

#endif // CONE_H
