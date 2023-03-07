//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"
#include "Physics/GJK.h"

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
AddStandardSandBox
====================================================
*/
void AddStandardSandBox( std::vector< Body > & bodies ) {
    Body body;

    body.m_position = Vec3( 0, 0, 0 );
    body.m_orientation = Quat( 0, 0, 0, 1 );
    body.m_linearVelocity.Zero();
    body.m_angularVelocity.Zero();
    body.m_invMass = 0.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.5f;
    body.m_shape = new ShapeBox( g_boxGround, sizeof( g_boxGround ) / sizeof( Vec3 ) );
    bodies.push_back( body );

    body.m_position = Vec3( 50, 0, 0 );
    body.m_orientation = Quat( 0, 0, 0, 1 );
    body.m_linearVelocity.Zero();
    body.m_angularVelocity.Zero();
    body.m_invMass = 0.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.0f;
    body.m_shape = new ShapeBox( g_boxWall0, sizeof( g_boxWall0 ) / sizeof( Vec3 ) );
    bodies.push_back( body );

    body.m_position = Vec3(-50, 0, 0 );
    body.m_orientation = Quat( 0, 0, 0, 1 );
    body.m_linearVelocity.Zero();
    body.m_angularVelocity.Zero();
    body.m_invMass = 0.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.0f;
    body.m_shape = new ShapeBox( g_boxWall0, sizeof( g_boxWall0 ) / sizeof( Vec3 ) );
    bodies.push_back( body );

    body.m_position = Vec3( 0, 25, 0 );
    body.m_orientation = Quat( 0, 0, 0, 1 );
    body.m_linearVelocity.Zero();
    body.m_angularVelocity.Zero();
    body.m_invMass = 0.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.0f;
    body.m_shape = new ShapeBox( g_boxWall1, sizeof( g_boxWall1 ) / sizeof( Vec3 ) );
    bodies.push_back( body );

    body.m_position = Vec3( 0,-25, 0 );
    body.m_orientation = Quat( 0, 0, 0, 1 );
    body.m_linearVelocity.Zero();
    body.m_angularVelocity.Zero();
    body.m_invMass = 0.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.0f;
    body.m_shape = new ShapeBox( g_boxWall1, sizeof( g_boxWall1 ) / sizeof( Vec3 ) );
    bodies.push_back( body );
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
    TestSignedVolumeProjection();

    Body body;

    body.m_position = Vec3( 10, 0, 3 );
    body.m_orientation = Quat( 0, 0, 0, 1 );
    body.m_linearVelocity = Vec3( -100, 0, 0 );
    body.m_angularVelocity = Vec3( 0.0f, 0.0f, 0.0f );
    body.m_invMass = 1.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.5f;
    body.m_shape = new ShapeSphere( 0.5f );
    m_bodies.push_back( body );

    body.m_position = Vec3( -10, 0, 3 );
    body.m_orientation = Quat( 0, 0, 0, 1 );
    body.m_linearVelocity = Vec3( 100, 0, 0 );
    body.m_angularVelocity = Vec3( 0, 10, 0 );
    body.m_invMass = 1.0f;
    body.m_elasticity = 0.5f;
    body.m_friction = 0.5f;
    body.m_shape = new ShapeConvex( g_diamond, sizeof( g_diamond ) / sizeof( Vec3 ) );
    m_bodies.push_back( body );

    // Dynamic Spheres
    for ( int x = 0; x < 6; x++ ) {
        for ( int y = 0; y < 6; y++ ) {
            float radius = 0.5f;
            float xx = float( x - 1 ) * radius * 1.5f;
            float yy = float( y - 1 ) * radius * 1.5f;
            body.m_position = Vec3( xx, yy, 10.0f );
            body.m_orientation = Quat( 0, 0, 0, 1 );
            body.m_linearVelocity.Zero();
            body.m_invMass = 1.0f;
            body.m_elasticity = 0.5f;
            body.m_friction = 0.5f;
            body.m_shape = new ShapeSphere( radius );
            m_bodies.push_back( body );
        }
    }

    // Dynamic boxes
    Vec3 boxVert[] = {
            Vec3(-1,-1, -1 ),
            Vec3( 1,-1, -1 ),
            Vec3(-1, 1, -1 ),
            Vec3( 1, 1, -1 ),

            Vec3(-1,-1,  1 ),
            Vec3( 1,-1,  1 ),
            Vec3(-1, 1,  1 ),
            Vec3( 1, 1,  1 ),
    };
    for (int i = 0; i < 8; i++)
    {
        boxVert[i] *= 0.5f;
    }
    for ( int x = 0; x < 6; x++ ) {
        for ( int y = 0; y < 6; y++ ) {
            float radius = 1.0f;
            float xx = float( x - 1 ) * radius * 1.5f;
            float yy = float( y - 1 ) * radius * 1.5f;
            body.m_position = Vec3( xx, yy, 20.0f );
            body.m_orientation = Quat( 0, 0, 0, 1 );
            body.m_linearVelocity.Zero();
            body.m_invMass = 1.0f;
            body.m_elasticity = 0.5f;
            body.m_friction = 0.5f;
            body.m_shape = new ShapeBox( boxVert, 8 );
            m_bodies.push_back( body );
        }
    }

    AddStandardSandBox( m_bodies );
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

    //
    // Broad Phase
    //
    std::vector<collisionPair_t> collisionPairs;
    BroadPhase(m_bodies.data(), (int)m_bodies.size(), collisionPairs, dt_sec);

    //
    // Narrow Phase (perform actual collision detection)
    //
    int numContacts = 0 ;
    const int maxContacts = m_bodies.size() * m_bodies.size() ;
    contact_t* contacts = (contact_t*)alloca(sizeof(contact_t) * maxContacts) ;
    for (int i = 0; i < collisionPairs.size(); i++)
    {
        const collisionPair_t& pair = collisionPairs[i];
        Body* bodyA = &m_bodies[pair.a];
        Body* bodyB = &m_bodies[pair.b];

        // Skip body pairs with infinite mass
        if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
            continue;

        contact_t contact;
        if (Intersect(bodyA, bodyB, dt_sec, contact))
        {
            contacts[numContacts] = contact;
            numContacts++;
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