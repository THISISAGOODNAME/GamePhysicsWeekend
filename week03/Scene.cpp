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

    for ( int i = 0; i < m_constraints.size(); i++ ) {
        delete m_constraints[ i ];
    }
    m_constraints.clear();

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
    Body body;

    //
    // Build a joint
    //
    if (false)
    {
        body.m_position = Vec3(0, 0, 5);
        body.m_orientation = Quat(0, 0, 0, 1);
        body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall)/sizeof(Vec3));
        body.m_invMass = 0.0f;
        body.m_elasticity = 1.0f;
        m_bodies.push_back(body);
        Body* bodyA = &m_bodies[m_bodies.size() - 1];

        body.m_position = Vec3(1, 0, 5);
        body.m_orientation = Quat(0, 0, 0, 1);
        body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall)/sizeof(Vec3));
        body.m_invMass = 1.0f;
        body.m_elasticity = 1.0f;
        m_bodies.push_back(body);
        Body* bodyB = &m_bodies[m_bodies.size() - 1];

        const Vec3 jointWorldSpaceAnchor = bodyA->m_position;

        ConstraintDistance* joint = new ConstraintDistance();

        joint->m_bodyA = bodyA;
        joint->m_anchorA = joint->m_bodyA->WorldSpaceToBodySpace(jointWorldSpaceAnchor);

        joint->m_bodyB = bodyB;
        joint->m_anchorB = joint->m_bodyB->WorldSpaceToBodySpace(jointWorldSpaceAnchor);
        m_constraints.push_back(joint);
    }

    //
    // Build a chain for funsies
    //
    if (false)
    {
        const int numJoints = 5;
        for ( int i = 0; i < numJoints; i++ ) {
            if ( i == 0 ) {
                body.m_position = Vec3( 0.0f, 5.0f, (float)numJoints + 3.0f );
                body.m_orientation = Quat( 0, 0, 0, 1 );
                body.m_shape = new ShapeBox( g_boxSmall, sizeof( g_boxSmall ) / sizeof( Vec3 ) );
                body.m_invMass = 0.0f;
                body.m_elasticity = 1.0f;
                m_bodies.push_back( body );
            } else {
                body.m_invMass = 1.0f;
            }

            body.m_linearVelocity = Vec3( 0, 0, 0 );

            Body * bodyA = &m_bodies[ m_bodies.size() - 1 ];
            const Vec3 jointWorldSpaceAnchor	= bodyA->m_position;

            ConstraintDistance * joint = new ConstraintDistance();

            joint->m_bodyA			= &m_bodies[ m_bodies.size() - 1 ];
            joint->m_anchorA		= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

            body.m_position = joint->m_bodyA->m_position + Vec3( 1, 0, 0 );
            body.m_orientation = Quat( 0, 0, 0, 1 );
            body.m_shape = new ShapeBox( g_boxSmall, sizeof( g_boxSmall ) / sizeof( Vec3 ) );
            body.m_invMass = 1.0f;
            body.m_elasticity = 1.0f;
            m_bodies.push_back( body );

            joint->m_bodyB			= &m_bodies[ m_bodies.size() - 1 ];
            joint->m_anchorB		= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

            m_constraints.push_back( joint );
        }
    }

    //
    //	Stack of Boxes
    //
    if (false)
    {
        const int stackHeight = 5;
        for ( int x = 0; x < 1; x++ ) {
            for ( int y = 0; y < 1; y++ ) {
                for ( int z = 0; z < stackHeight; z++ ) {
                    float offset = ( ( z & 1 ) == 0 ) ? 0.0f : 0.15f;
                    float xx = (float)x + offset;
                    float yy = (float)y + offset;
                    float delta = 0.04f;
                    float scaleHeight = 2.0f + delta;
                    float deltaHeight = 1.0f + delta;
                    body.m_position = Vec3( (float)xx * scaleHeight, (float)yy * scaleHeight, deltaHeight + (float)z * scaleHeight );
                    body.m_orientation = Quat( 0, 0, 0, 1 );
                    body.m_shape = new ShapeBox( g_boxUnit, sizeof( g_boxUnit ) / sizeof( Vec3 ) );
                    body.m_invMass = 1.0f;
                    body.m_elasticity = 0.5f;
                    body.m_friction = 0.5f;
                    m_bodies.push_back( body );
                }
            }
        }
    }

    //
    //	Build a ragdoll
    //
    if (true)
    {
        Vec3 offset = Vec3( -5, 0, 0 );

        // head
        body.m_position = Vec3( 0, 0, 5.5f ) + offset;
        body.m_orientation = Quat( 0, 0, 0, 1 );
        body.m_shape = new ShapeBox( g_boxSmall, sizeof( g_boxSmall ) / sizeof( Vec3 ) );
        body.m_invMass = 2.0f;
        body.m_elasticity = 1.0f;
        body.m_friction = 1.0f;
        m_bodies.push_back( body );

        // torso
        body.m_position = Vec3( 0, 0, 4 ) + offset;
        body.m_orientation = Quat( 0, 0, 0, 1 );
        body.m_shape = new ShapeBox( g_boxBody, sizeof( g_boxBody ) / sizeof( Vec3 ) );
        body.m_invMass = 0.5f;
        body.m_elasticity = 1.0f;
        body.m_friction = 1.0f;
        m_bodies.push_back( body );

        // left arm
        body.m_position = Vec3( 0.0f, 2.0f, 4.75f ) + offset;
        body.m_orientation = Quat( Vec3( 0, 0, 1 ), -3.1415f / 2.0f );
        body.m_shape = new ShapeBox( g_boxLimb, sizeof( g_boxLimb ) / sizeof( Vec3 ) );
        body.m_invMass = 1.0f;
        body.m_elasticity = 1.0f;
        body.m_friction = 1.0f;
        m_bodies.push_back( body );

        // right arm
        body.m_position = Vec3( 0.0f, -2.0f, 4.75f ) + offset;
        body.m_orientation = Quat( Vec3( 0, 0, 1 ), 3.1415f / 2.0f );
        body.m_shape = new ShapeBox( g_boxLimb, sizeof( g_boxLimb ) / sizeof( Vec3 ) );
        body.m_invMass = 1.0f;
        body.m_elasticity = 1.0f;
        body.m_friction = 1.0f;
        m_bodies.push_back( body );

        // left leg
        body.m_position = Vec3( 0.0f, 1.0f, 2.5f ) + offset;
        body.m_orientation = Quat( Vec3( 0, 1, 0 ), 3.1415f / 2.0f );
        body.m_shape = new ShapeBox( g_boxLimb, sizeof( g_boxLimb ) / sizeof( Vec3 ) );
        body.m_invMass = 1.0f;
        body.m_elasticity = 1.0f;
        body.m_friction = 1.0f;
        m_bodies.push_back( body );

        // right leg
        body.m_position = Vec3( 0.0f, -1.0f, 2.5f ) + offset;
        body.m_orientation = Quat( Vec3( 0, 1, 0 ), 3.1415f / 2.0f );
        body.m_shape = new ShapeBox( g_boxLimb, sizeof( g_boxLimb ) / sizeof( Vec3 ) );
        body.m_invMass = 1.0f;
        body.m_elasticity = 1.0f;
        body.m_friction = 1.0f;
        m_bodies.push_back( body );

        const int idxHead = 0;
        const int idxTorso = 1;
        const int idxArmLeft = 2;
        const int idxArmRight = 3;
        const int idxLegLeft = 4;
        const int idxLegRight = 5;

        // Neck
        {
            ConstraintHingeQuatLimited * joint = new ConstraintHingeQuatLimited();
            joint->m_bodyA = &m_bodies[ idxHead ];
            joint->m_bodyB = &m_bodies[ idxTorso ];

            const Vec3 jointWorldSpaceAnchor	= joint->m_bodyA->m_position + Vec3( 0, 0, -0.5f );
            joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
            joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

            joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint( Vec3( 0, 1, 0 ) );

            // Set the initial relative orientation
            joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

            m_constraints.push_back( joint );
        }

        // Shoulder Left
        {
            ConstraintConstantVelocityLimited * joint = new ConstraintConstantVelocityLimited();
            joint->m_bodyB = &m_bodies[ idxArmLeft ];
            joint->m_bodyA = &m_bodies[ idxTorso ];

            const Vec3 jointWorldSpaceAnchor	= joint->m_bodyB->m_position + Vec3( 0, -1.0f, 0.0f );
            joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
            joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

            joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint( Vec3( 0, 1, 0 ) );

            // Set the initial relative orientation
            joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

            m_constraints.push_back( joint );
        }

        // Shoulder Right
        {
            ConstraintConstantVelocityLimited * joint = new ConstraintConstantVelocityLimited();
            joint->m_bodyB = &m_bodies[ idxArmRight ];
            joint->m_bodyA = &m_bodies[ idxTorso ];

            const Vec3 jointWorldSpaceAnchor	= joint->m_bodyB->m_position + Vec3( 0, 1.0f, 0.0f );
            joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
            joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

            joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint( Vec3( 0, -1, 0 ) );

            // Set the initial relative orientation
            joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

            m_constraints.push_back( joint );
        }

        // Hip Left
        {
            ConstraintHingeQuatLimited * joint = new ConstraintHingeQuatLimited();
            joint->m_bodyB = &m_bodies[ idxLegLeft ];
            joint->m_bodyA = &m_bodies[ idxTorso ];

            const Vec3 jointWorldSpaceAnchor	= joint->m_bodyB->m_position + Vec3( 0, 0, 0.5f );
            joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
            joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

            joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint( Vec3( 0, 1, 0 ) );

            // Set the initial relative orientation
            joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

            m_constraints.push_back( joint );
        }

        // Hip Right
        {
            ConstraintHingeQuatLimited * joint = new ConstraintHingeQuatLimited();
            joint->m_bodyB = &m_bodies[ idxLegRight ];
            joint->m_bodyA = &m_bodies[ idxTorso ];

            const Vec3 jointWorldSpaceAnchor	= joint->m_bodyB->m_position + Vec3( 0, 0, 0.5f );
            joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
            joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

            joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint( Vec3( 0, 1, 0 ) );

            // Set the initial relative orientation
            joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

            m_constraints.push_back( joint );
        }
    }


    //
    //	Standard floor and walls
    //
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
    m_manifolds.RemoveExpired();

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
    // Broad Phase (build potential collision pairs)
    //
    std::vector<collisionPair_t> collisionPairs;
    BroadPhase(m_bodies.data(), (int)m_bodies.size(), collisionPairs, dt_sec);

    //
    // Narrow Phase (perform actual collision detection)
    //
    int numContacts = 0 ;
    contact_t* contacts = (contact_t*)alloca(sizeof(contact_t) * collisionPairs.size()) ;
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
            if ( 0.0f == contact.timeOfImpact)
            {
                // static contact
                m_manifolds.AddContact(contact);
            }
            else
            {
                // ballistic contact
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

    //
    //	Solve Constraints
    //
    for (int i = 0; i < m_constraints.size(); i++)
    {
        m_constraints[i]->PreSolve(dt_sec);
    }
    m_manifolds.PreSolve(dt_sec);
    const int maxIters = 5;
    for ( int iters = 0; iters < maxIters; iters++ ) {
        for ( int i = 0; i < m_constraints.size(); i++ ) {
            m_constraints[ i ]->Solve();
        }
        m_manifolds.Solve();
    }
    for (int i = 0; i < m_constraints.size(); i++)
    {
        m_constraints[i]->PostSolve();
    }
    m_manifolds.PostSolve();

    //
    // Apply ballistic impulses
    //
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