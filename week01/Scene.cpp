//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	Body body;
	body.m_position = Vec3( -3, 0, 3 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeSphere( 1.0f );
    body.m_linearVelocity = Vec3(100, 0,0);
    body.m_invMass = 1.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.5f;
	m_bodies.push_back( body );

    body.m_position = Vec3( 0, 0, 3 );
    body.m_orientation = Quat( 0, 0, 0, 1 );
    body.m_shape = new ShapeSphere( 1.0f );
    body.m_linearVelocity = Vec3(0, 0,0);
    body.m_invMass = 0.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.5f;
    m_bodies.push_back( body );

    // Add a ”ground” sphere that won’t fall under the influence of gravity
	body.m_position = Vec3( 0, 0, -1000 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeSphere( 1000.0f );
    body.m_linearVelocity = Vec3(0, 0,0);
    body.m_invMass = 0.0f;
    body.m_elasticity = 1.0f;
    body.m_friction = 0.5f;
	m_bodies.push_back( body );
}

/*
====================================================
CompareContacts
====================================================
*/
int CompareContacts( const void * p1, const void * p2 )
{
    contact_t a = *(contact_t*)p1;
    contact_t b = *(contact_t*)p2;

    if (a.timeOfImpact < b.timeOfImpact)
        return -1;

    if (a.timeOfImpact > b.timeOfImpact)
        return 1;

    return 0;
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) {
    // Gravity impulse
    for (int i = 0; i < m_bodies.size(); i++)
    {
        Body* body = &m_bodies[i];

        // Gravity needs to be an impulse
        // I = dp , F = dp/ dt => dp = F * dt => I = F * dt
        // F = mgs
        float mass = 1.0f / body->m_invMass;
        Vec3 impulseGravity = GRAVITY * mass * dt_sec;
        body->ApplyImpulseLinear(impulseGravity);
    }

    int numContacts = 0 ;
    const int maxContacts = m_bodies.size() * m_bodies.size() ;
    contact_t* contacts = (contact_t*)alloca(sizeof(contact_t) * maxContacts) ;
    for (int i = 0 ; i < m_bodies.size(); i++ )
    {
        for ( int j = i + 1 ; j < m_bodies.size() ; j++)
        {
            Body* bodyA = &m_bodies[i];
            Body* bodyB = &m_bodies[j];

            // Skip body pairs with infinite mass
            if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass ) {
                continue ;
            }

            contact_t contact;
            if (Intersect(bodyA, bodyB, dt_sec, contact))
            {
                contacts[numContacts] = contact;
                numContacts++;
            }
        }
    }

    // Sort the times of impact from earliest to latest
    if (numContacts > 1 )
    {
        qsort(contacts, numContacts, sizeof(contact_t) , CompareContacts);
    }

    float accumulatedTime = 0.0f;
    for (int i = 0; i < numContacts; i++)
    {
        contact_t& contact = contacts[i];
        const float dt = contact.timeOfImpact - accumulatedTime;

        Body* bodyA = contact.bodyA;
        Body* bodyB = contact.bodyB;

        // Skip body pairs with infinite mass
        if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
            continue;

        // Position update
        for (int j = 0 ; j < m_bodies.size() ; j++)
        {
            m_bodies[j].Update(dt);
        }

        ResolveContact(contact);
        accumulatedTime += dt;
    }

    // Update the positions for the rest of this frame’s time
    const float timeRemaining = dt_sec - accumulatedTime;
    if (timeRemaining > 0.0f )
    {
        for (int i = 0 ; i < m_bodies.size() ; i++)
        {
            m_bodies[i].Update(timeRemaining);
        }
    }
}