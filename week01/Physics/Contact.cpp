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
	// TODO: Add Code
    Body* bodyA = contact.bodyA;
    Body* bodyB = contact.bodyB;

    const float invMassA = bodyA->m_invMass;
    const float invMassB = bodyB->m_invMass;

    // Calculate the collision impulse
    const Vec3& n = contact.normal;
    const Vec3 vab = bodyA->m_linearVelocity - bodyB->m_linearVelocity;
    const float impulseJ = -2.0f * vab.Dot(n) / (invMassA + invMassB);
    const Vec3 vectorImpulseJ = n * impulseJ;

    bodyA->ApplyImpulseLinear(vectorImpulseJ * 1.0f);
    bodyB->ApplyImpulseLinear(vectorImpulseJ * -1.0f);

    // Let’s also move our colliding objects to just outside of each other
    const float tA = bodyA->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);
    const float tB = bodyB->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);

    const Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
    bodyA->m_position += ds * tA;
    bodyB->m_position -= ds * tB;
}