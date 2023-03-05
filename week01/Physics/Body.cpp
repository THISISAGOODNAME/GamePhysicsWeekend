//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
    m_position( 0.0f ),
    m_orientation( 0.0f, 0.0f, 0.0f, 1.0f ),
    m_shape( NULL ),
    m_linearVelocity(0.0f)
{
}

Vec3 Body::GetCenterOfMassWorldSpace() const
{
    const Vec3 conterOfMass = m_shape->GetCenterOfMass();
    const Vec3 pos = m_position + m_orientation.RotatePoint(conterOfMass);
    return pos;
}

Vec3 Body::GetCenterOfMassModelSpace() const
{
    const Vec3 centerOfMass = m_shape->GetCenterOfMass();
    return centerOfMass;
}

Vec3 Body::WorldSpaceToBodySpace(const Vec3 & pt) const
{
    Vec3 tmp = pt - GetCenterOfMassWorldSpace();
    Quat inverseOrient = m_orientation.Inverse();
    Vec3 bodySpacePos = inverseOrient.RotatePoint(tmp);
    return bodySpacePos;
}

Vec3 Body::BodySpaceToWorldSpace(const Vec3 &pt) const
{
    Vec3 worldSpacePos = GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(pt);
    return worldSpacePos;
}

void Body::ApplyImpulseLinear(const Vec3 &impulse)
{
    if (0.0f == m_invMass)
        return;

    // p = mv
    // dp = m dv = J
    // => j = J / m
    m_linearVelocity += impulse * m_invMass;
}
