//
//  GJK.cpp
//
#include "GJK.h"

/*
================================================================================================

Signed Volumes

================================================================================================
*/

/*
================================
SignedVolume1D
================================
*/
Vec2 SignedVolume1D( const Vec3 & s1, const Vec3 & s2 ) {
    Vec3 ab = s2 - s1;	// Ray from a to b
    Vec3 ap = Vec3( 0.0f ) - s1;	// Ray from a to origin
    Vec3 p0 = s1 + ab * ab.Dot( ap ) / ab.GetLengthSqr();	// projection of the origin onto the line

    // Choose the axis with the greatest difference/length
    int idx = 0;
    float mu_max = 0;
    for ( int i = 0; i < 3; i++ ) {
        float mu = s2[ i ] - s1[ i ];
        if ( mu * mu > mu_max * mu_max ) {
            mu_max = mu;
            idx = i;
        }
    }

    // Project the simplex points and projected origin onto the axis with greatest length
    const float a = s1[ idx ];
    const float b = s2[ idx ];
    const float p = p0[ idx ];

    // Get the signed distance from a to p and from p to b
    const float C1 = p - a;
    const float C2 = b - p;

    // if p is between [a,b]
    if ( ( p > a && p < b ) || ( p > b && p < a ) ) {
        Vec2 lambdas;
        lambdas[ 0 ] = C2 / mu_max;
        lambdas[ 1 ] = C1 / mu_max;
        return lambdas;
    }

    // if p is on the far side of a
    if ( ( a <= b && p <= a ) || ( a >= b && p >= a ) ) {
        return Vec2( 1.0f, 0.0f );
    }

    // p must be on the far side of b
    return Vec2( 0.0f, 1.0f );
}

/*
================================
CompareSigns
================================
*/
int CompareSigns( float a, float b ) {
    if ( a > 0.0f && b > 0.0f ) {
        return 1;
    }
    if ( a < 0.0f && b < 0.0f ) {
        return 1;
    }
    return 0;
}

/*
================================
SignedVolume2D
================================
*/
Vec3 SignedVolume2D( const Vec3 & s1, const Vec3 & s2, const Vec3 & s3 ) {
    Vec3 normal = ( s2 - s1 ).Cross( s3 - s1 );
    Vec3 p0 = normal * s1.Dot( normal ) / normal.GetLengthSqr();

    // Find the axis with the greatest projected area
    int idx = 0;
    float area_max = 0;
    for ( int i = 0; i < 3; i++ ) {
        int j = ( i + 1 ) % 3;
        int k = ( i + 2 ) % 3;

        Vec2 a = Vec2( s1[ j ], s1[ k ] );
        Vec2 b = Vec2( s2[ j ], s2[ k ] );
        Vec2 c = Vec2( s3[ j ], s3[ k ] );
        Vec2 ab = b - a;
        Vec2 ac = c - a;

        float area = ab.x * ac.y - ab.y * ac.x;
        if ( area * area > area_max * area_max ) {
            idx = i;
            area_max = area;
        }
    }

    // Project onto the appropriate axis
    int x = ( idx + 1 ) % 3;
    int y = ( idx + 2 ) % 3;
    Vec2 s[ 3 ];
    s[ 0 ] = Vec2( s1[ x ], s1[ y ] );
    s[ 1 ] = Vec2( s2[ x ], s2[ y ] );
    s[ 2 ] = Vec2( s3[ x ], s3[ y ] );
    Vec2 p = Vec2( p0[ x ], p0[ y ] );

    // Get the sub-areas of the triangles formed from the projected origin and the edges
    Vec3 areas;
    for ( int i = 0; i < 3; i++ ) {
        int j = ( i + 1 ) % 3;
        int k = ( i + 2 ) % 3;

        Vec2 a = p;
        Vec2 b = s[ j ];
        Vec2 c = s[ k ];
        Vec2 ab = b - a;
        Vec2 ac = c - a;

        areas[ i ] = ab.x * ac.y - ab.y * ac.x;
    }

    // If the projected origin is inside the triangle, then return the barycentric points
    if ( CompareSigns( area_max, areas[ 0 ] ) > 0 && CompareSigns( area_max, areas[ 1 ] ) > 0 && CompareSigns( area_max, areas[ 2 ] ) > 0 ) {
        Vec3 lambdas = areas / area_max;
        return lambdas;
    }

    // If we make it here, then we need to project onto the edges and determine the closest point
    float dist = 1e10;
    Vec3 lambdas = Vec3( 1, 0, 0 );
    for ( int i = 0; i < 3; i++ ) {
        int k = ( i + 1 ) % 3;
        int l = ( i + 2 ) % 3;

        Vec3 edgesPts[ 3 ];
        edgesPts[ 0 ] = s1;
        edgesPts[ 1 ] = s2;
        edgesPts[ 2 ] = s3;

        Vec2 lambdaEdge = SignedVolume1D( edgesPts[ k ], edgesPts[ l ] );
        Vec3 pt = edgesPts[ k ] * lambdaEdge[ 0 ] + edgesPts[ l ] * lambdaEdge[ 1 ];
        if ( pt.GetLengthSqr() < dist ) {
            dist = pt.GetLengthSqr();
            lambdas[ i ] = 0;
            lambdas[ k ] = lambdaEdge[ 0 ];
            lambdas[ l ] = lambdaEdge[ 1 ];
        }
    }

    return lambdas;
}

/*
================================
SignedVolume3D
================================
*/
Vec4 SignedVolume3D( const Vec3 & s1, const Vec3 & s2, const Vec3 & s3, const Vec3 & s4 ) {
    Mat4 M;
    M.rows[ 0 ] = Vec4( s1.x, s2.x, s3.x, s4.x );
    M.rows[ 1 ] = Vec4( s1.y, s2.y, s3.y, s4.y );
    M.rows[ 2 ] = Vec4( s1.z, s2.z, s3.z, s4.z );
    M.rows[ 3 ] = Vec4( 1.0f, 1.0f, 1.0f, 1.0f );

    Vec4 C4;
    C4[ 0 ] = M.Cofactor( 3, 0 );
    C4[ 1 ] = M.Cofactor( 3, 1 );
    C4[ 2 ] = M.Cofactor( 3, 2 );
    C4[ 3 ] = M.Cofactor( 3, 3 );

    const float detM = C4[ 0 ] + C4[ 1 ] + C4[ 2 ] + C4[ 3 ];

    // If the barycentric coordinates put the origin inside the simplex, then return them
    if ( CompareSigns( detM, C4[ 0 ] ) > 0 && CompareSigns( detM, C4[ 1 ] ) > 0 && CompareSigns( detM, C4[ 2 ] ) > 0 && CompareSigns( detM, C4[ 3 ] ) > 0 ) {
        Vec4 lambdas = C4 * ( 1.0f / detM );
        return lambdas;
    }

    // If we get here, then we need to project the origin onto the faces and determine the closest one
    Vec4 lambdas;
    float dist = 1e10;
    for ( int i = 0; i < 4; i++ ) {
        int j = ( i + 1 ) % 4;
        int k = ( i + 2 ) % 4;

        Vec3 facePts[ 4 ];
        facePts[ 0 ] = s1;
        facePts[ 1 ] = s2;
        facePts[ 2 ] = s3;
        facePts[ 3 ] = s4;

        Vec3 lambdasFace = SignedVolume2D( facePts[ i ], facePts[ j ], facePts[ k ] );
        Vec3 pt = facePts[ i ] * lambdasFace[ 0 ] + facePts[ j ] * lambdasFace[ 1 ] + facePts[ k ] * lambdasFace[ 2 ];
        if ( pt.GetLengthSqr() < dist ) {
            dist = pt.GetLengthSqr();
            lambdas.Zero();
            lambdas[ i ] = lambdasFace[ 0 ];
            lambdas[ j ] = lambdasFace[ 1 ];
            lambdas[ k ] = lambdasFace[ 2 ];
        }
    }

    return lambdas;
}

/*
================================
TestSignedVolumeProjection
================================
*/
void TestSignedVolumeProjection() {
    const Vec3 orgPts[ 4 ] = {
            Vec3( 0, 0, 0 ),
            Vec3( 1, 0, 0 ),
            Vec3( 0, 1, 0 ),
            Vec3( 0, 0, 1 ),
    };
    Vec3 pts[ 4 ];
    Vec4 lambdas;
    Vec3 v;

    for ( int i = 0; i < 4; i++ ) {
        pts[ i ] = orgPts[ i ] + Vec3( 1, 1, 1 );
    }
    lambdas = SignedVolume3D( pts[ 0 ], pts[ 1 ], pts[ 2 ], pts[ 3 ] );
    v.Zero();
    for ( int i = 0; i < 4; i++ ) {
        v += pts[ i ] * lambdas[ i ];
    }
    printf( "lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
            lambdas.x, lambdas.y, lambdas.z, lambdas.w,
            v.x, v.y, v.z
    );

    for ( int i = 0; i < 4; i++ ) {
        pts[ i ] = orgPts[ i ] + Vec3( -1, -1, -1 ) * 0.25f;
    }
    lambdas = SignedVolume3D( pts[ 0 ], pts[ 1 ], pts[ 2 ], pts[ 3 ] );
    v.Zero();
    for ( int i = 0; i < 4; i++ ) {
        v += pts[ i ] * lambdas[ i ];
    }
    printf( "lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
            lambdas.x, lambdas.y, lambdas.z, lambdas.w,
            v.x, v.y, v.z
    );

    for ( int i = 0; i < 4; i++ ) {
        pts[ i ] = orgPts[ i ] + Vec3( -1, -1, -1 );
    }
    lambdas = SignedVolume3D( pts[ 0 ], pts[ 1 ], pts[ 2 ], pts[ 3 ] );
    v.Zero();
    for ( int i = 0; i < 4; i++ ) {
        v += pts[ i ] * lambdas[ i ];
    }
    printf( "lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
            lambdas.x, lambdas.y, lambdas.z, lambdas.w,
            v.x, v.y, v.z
    );

    for ( int i = 0; i < 4; i++ ) {
        pts[ i ] = orgPts[ i ] + Vec3( 1, 1, -0.5f );
    }
    lambdas = SignedVolume3D( pts[ 0 ], pts[ 1 ], pts[ 2 ], pts[ 3 ] );
    v.Zero();
    for ( int i = 0; i < 4; i++ ) {
        v += pts[ i ] * lambdas[ i ];
    }
    printf( "lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
            lambdas.x, lambdas.y, lambdas.z, lambdas.w,
            v.x, v.y, v.z
    );

    pts[ 0 ] = Vec3( 51.1996613f, 26.1989613f, 1.91339576f );
    pts[ 1 ] = Vec3( -51.0567360f, -26.0565681f, -0.436143428f );
    pts[ 2 ] = Vec3( 50.8978920f, -24.1035538f, -1.04042661f );
    pts[ 3 ] = Vec3( -49.1021080f, 25.8964462f, -1.04042661f );
    lambdas = SignedVolume3D( pts[ 0 ], pts[ 1 ], pts[ 2 ], pts[ 3 ] );
    v.Zero();
    for ( int i = 0; i < 4; i++ ) {
        v += pts[ i ] * lambdas[ i ];
    }
    printf( "lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
            lambdas.x, lambdas.y, lambdas.z, lambdas.w,
            v.x, v.y, v.z
    );
}

/*
================================
GJK_DoesIntersect
================================
*/
bool GJK_DoesIntersect( const Body * bodyA, const Body * bodyB ) {
	// TODO: Add code

	return false;
}

/*
================================
GJK_ClosestPoints
================================
*/
void GJK_ClosestPoints( const Body * bodyA, const Body * bodyB, Vec3 & ptOnA, Vec3 & ptOnB ) {
	// TODO: Add code
}

/*
================================
GJK_DoesIntersect
================================
*/
bool GJK_DoesIntersect( const Body * bodyA, const Body * bodyB, const float bias, Vec3 & ptOnA, Vec3 & ptOnB ) {
	// TODO: Add code

	return false;
}