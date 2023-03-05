//
//  Scene.h
//
#pragma once
#include <vector>

#include "Physics/Shapes.h"
#include "Physics/Body.h"
#include "Physics/Constraints.h"
#include "Physics/Manifold.h"

/*
====================================================
Scene
====================================================
*/

const static Vec3 GRAVITY = Vec3(0.0f, 0.0f, -10.0f);

class Scene {
public:
	Scene() { m_bodies.reserve( 128 ); }
	~Scene();

	void Reset();
	void Initialize();
	void Update( const float dt_sec );	

	std::vector< Body > m_bodies;
	std::vector< Constraint * >	m_constraints;
	ManifoldCollector m_manifolds;
};

