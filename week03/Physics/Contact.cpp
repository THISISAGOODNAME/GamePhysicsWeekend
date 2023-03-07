//
//  Contact.cpp
//
#include "Contact.h"

/*
====================================================
ResolveContact
====================================================
*/
void ResolveContact( contact_t & contact ) {
    Body* bodyA = contact.bodyA;
    Body* bodyB = contact.bodyB;

    const Vec3 ptOnA = contact.ptOnA_WorldSpace;
    const Vec3 ptOnB = contact.ptOnB_WorldSpace;

    const float invMassA = bodyA->m_invMass;
    const float invMassB = bodyB->m_invMass;

    const Mat3 invWorldInertiaA = bodyA->GetInverseInertiaTensorWorldSpace();
    const Mat3 invWorldInertiaB = bodyB->GetInverseInertiaTensorWorldSpace();

    const float elasticityA = bodyA->m_elasticity;
    const float elasticityB = bodyB->m_elasticity;
    const float elasticity = elasticityA * elasticityB;

    const Vec3& n = contact.normal;

    const Vec3 ra = ptOnA - bodyA->GetCenterOfMassWorldSpace();
    const Vec3 rb = ptOnB - bodyB->GetCenterOfMassWorldSpace();

    const Vec3 anjularJA = (invWorldInertiaA * ra.Cross(n)).Cross(ra);
    const Vec3 anjularJB = (invWorldInertiaB * rb.Cross(n)).Cross(rb);
    const float angularFactor = (anjularJA + anjularJB).Dot(n);

    // Get the world space velocity of the motion and rotation
    const Vec3 velA = bodyA->m_linearVelocity + bodyA->m_angularVelocity.Cross(ra);
    const Vec3 velB = bodyB->m_linearVelocity + bodyB->m_angularVelocity.Cross(rb);

    // Calculate the collision impulse
    const Vec3 vab = velA - velB;
    const float impulseJ = (1.0f + elasticity) * vab.Dot(n) / (invMassA + invMassB + angularFactor);
    const Vec3 vectorImpulseJ = n * impulseJ;

    bodyA->ApplyImpulse(ptOnA, vectorImpulseJ * -1.0f);
    bodyB->ApplyImpulse(ptOnB, vectorImpulseJ * 1.0f);

    //
    // Calculate the impulse caused by friction
    //
    const float frictionA = bodyA->m_friction;
    const float frictionB = bodyB->m_friction;
    const float friction = frictionA * frictionB;

    // Find the normal direction of the velocity with respect to the normal of the collision
    const Vec3 velNorm = n * n.Dot( vab );

    // Find the tangent direction of the velocity with respect to the normal of the collision
    const Vec3 velTang = vab - velNorm;

    // Get the tangential velocities relative to the other body
    Vec3 relativeVelTang = velTang;
    relativeVelTang.Normalize();

    const Vec3 inertiaA = ( invWorldInertiaA * ra.Cross( relativeVelTang ) ).Cross( ra );
    const Vec3 inertiaB = ( invWorldInertiaB * rb.Cross( relativeVelTang ) ).Cross( rb );
    const float invInertia = ( inertiaA + inertiaB ).Dot( relativeVelTang );

    // Calculate the tangential impulse for friction
    const float reducedMass = 1.0f / ( bodyA->m_invMass + bodyB->m_invMass + invInertia );
    const Vec3 impulseFriction = velTang * reducedMass * friction;

    // Apply kinetic friction
    bodyA->ApplyImpulse(ptOnA, impulseFriction * -1.0f);
    bodyB->ApplyImpulse(ptOnB, impulseFriction * 1.0f);

    //
    // Let’s also move our colliding objects to just outside of each other
    // Only when TOI is 0.0f
    //
    if (0.0f == contact.timeOfImpact)
    {
        const Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
        const float tA = bodyA->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);
        const float tB = bodyB->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);

        bodyA->m_position += ds * tA;
        bodyB->m_position -= ds * tB;
    }
}