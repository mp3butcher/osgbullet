/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009-2012 by Kenneth Mark Bryden
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *************** <auto-copyright.pl END do not edit this line> ***************/

#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/CreationRecord.h>
#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/World.h>
#include <osgbCollision/Utils.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgAnimation/RigGeometry>
#include <osgAnimation/Bone>
#include <osgAnimation/BoneMapVisitor>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <osg/Node>
#include <osg/MatrixTransform>
#include <osg/BoundingSphere>
#include <osg/Notify>
#include <osg/ref_ptr>
#include <osg/io_utils>
#include <osgbCollision/Utils.h>
#include <osgUtil/SmoothingVisitor>
#include <osgbDynamics/TripleBuffer.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <iostream>
#include <ConvexBuilder.h>
#include <osgUtil/Optimizer>
#include <osgbCollision/ComputeTriMeshVisitor.h>
#include <osg/TriangleFunctor>

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btConvexHullComputer.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btGeometryUtil.h"
#include <osg/PolygonMode>
#include <osg/Point>
#include <osg/CullFace>
using namespace osgbCollision;
namespace osgbDynamics
{

///motion state assuming drawable centered on their center of mass
class RigidBodyMotionState : public MotionState
{
public:
    virtual void setWorldTransform(const btTransform& worldTrans)
    {
        // Call the callback, if registered.
        if( _mscl.size() > 0 )
        {
            // Call only if position changed.
            const btVector3 delta( worldTrans.getOrigin() - _transform.getOrigin() );
            const btScalar eps( (btScalar)( 1e-5 ) );
            const bool quiescent( osg::equivalent( delta[ 0 ], btScalar(0.), eps ) &&
                                  osg::equivalent( delta[ 1 ], btScalar(0.), eps ) &&
                                  osg::equivalent( delta[ 2 ], btScalar(0.), eps ) );
            if( !quiescent )
            {
                MotionStateCallbackList::iterator it;
                for( it = _mscl.begin(); it != _mscl.end(); ++it )
                    (**it)( worldTrans );
            }
        }

        // _transform is the model-to-world transformation used to place collision shapes
        // in the physics simulation. Bullet queries this with getWorldTransform().
        _transform = worldTrans;

        if( _tb == NULL )
        {
            // setWorldTransformInternal( worldTrans );

            const osg::Matrix dt = osgbCollision::asOsgMatrix( worldTrans );
            /* const osg::Matrix col2ol = computeCOLocalToOsgLocal();
             const osg::Matrix t = col2ol * dt;*/

            if( _mt.valid() )
                _mt->setMatrix( dt );
            else if( _amt.valid() )
                _amt->setMatrix( dt );


        }
        else
        {
            char* addr( _tb->writeAddress() );
            if( addr == NULL )
            {
                osg::notify( osg::WARN ) << "MotionState: No TripleBuffer write address." << std::endl;
                return;
            }
            btScalar* fAddr = reinterpret_cast< btScalar* >( addr + _tbIndex );
            worldTrans.getOpenGLMatrix( fAddr );
        }
    }

};


/*
void PhysicalObject::operator()( osg::Node* node, osg::NodeVisitor* nv )
{

    if ( !_parentWorld)
    {
        FindParentVisitor fpv;
        node->accept(fpv);
        _parentWorld=fpv.foundWorld;
        addPhysicalObjectToParentWorld();
    }
    updatematrix(node,nv);
    traverse( node, nv );
}

PhysicalObject::~PhysicalObject() {}
*/
RigidBody::RigidBody():_parentWorld(0),_body(0) {}
RigidBody::~RigidBody()
{
    if(getRigidBody())
    {
        if(_parentWorld)_parentWorld->getDynamicsWorld()->removeRigidBody(getRigidBody());
        delete getRigidBody();
    }
}
RigidBody::RigidBody( const RigidBody& copy, const osg::CopyOp& copyop ) {}
/*const btRigidBody* RigidBody::getRigidBody()const
{
    const   osgbCollision::RefBulletObject<  btRigidBody >*  refptr=
        dynamic_cast<  const   osgbCollision::RefBulletObject<  btRigidBody >* >( getUserData() );

    return (refptr?refptr->get():0);
}

btRigidBody* RigidBody::getRigidBody()
{
    osgbCollision::RefBulletObject<  btRigidBody >*  refptr=
        dynamic_cast<     osgbCollision::RefBulletObject<  btRigidBody >* >( getUserData() );

    return (refptr?refptr->get():0);
}

void RigidBody::setRigidBody( btRigidBody *b )
{
    setUserData(new osgbCollision::RefBulletObject<btRigidBody>(b));
    //btRigidBody >* rb = dynamic_cast<        osgbCollision::RefBulletObject<  >* >();


}*/
void RigidBody::addJoint(Joint*p)
{
    for(std::vector< osg::ref_ptr<Joint> >::iterator i=_joints.begin(); i!=_joints.end(); i++)
        if(i->get() ==p)return;

    _joints.push_back(p);
    if(_parentWorld)_parentWorld->addJoint(p);
}
void RigidBody::removeJoint(Joint*p)
{
    for(std::vector< osg::ref_ptr<Joint> >::iterator i=_joints.begin(); i!=_joints.end(); i++)
    {
        if(i->get() ==p)
        {
            _joints.erase(i);
            if(_parentWorld)_parentWorld->removeJoint(p);
            return;
        }
    }
}

void RigidBody::operator()( osg::Node* node, osg::NodeVisitor* nv )
{

    if ( !_parentWorld)
    {
        WorldFinderVisitor fpv;
        node->accept(fpv);
        _parentWorld=fpv.getFoundWorld();
        //addPhysicalObjectToParentWorld(static_cast<osg::MatrixTransform*>(node));
        if(_body&&_parentWorld)
        {
            if(!dynamic_cast<osgbDynamics::RigidBodyMotionState*>(_body->getMotionState()))
            {
///_body is not compatible with osgBullet RigidBody so transform it

                btMotionState * ms=_body->getMotionState();
                btTransform trans;
                trans=_body->getWorldTransform();

                osgbDynamics::RigidBodyMotionState *osgms= new osgbDynamics::RigidBodyMotionState();
                osg::MatrixTransform* mat=dynamic_cast<osg::MatrixTransform*>(node);
                osgms->setTransform(mat);
                osgms->setWorldTransform(trans);
                osgms->setParentTransform(mat->getMatrix());
                _body->setMotionState(osgms);
                if (ms)delete ms;


            }

            ///addPhysicalObject
            if(_parentWorld->getDynamicsWorld()->getCollisionObjectArray().findLinearSearch(_body) == _parentWorld->getDynamicsWorld()->getCollisionObjectArray().size())
            {
                _parentWorld->getDynamicsWorld()->addRigidBody(_body);
                // std::cerr<<" num joints"<<_joints.size()<<std::endl;
                for(std::vector< osg::ref_ptr<Joint> >::iterator i=_joints.begin(); i!=_joints.end(); i++)
                {
                    _parentWorld->addJoint(*i);
                }
            }
        }
    }
//   updatematrix(node,nv);
    traverse( node, nv );
}
/*
void RigidBody::operator()(osg::Node*node,osg::NodeVisitor *nv)
{

    //not sure update visitor will be applied if(nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR)
    if ( !_parentWorld)
    {
        if(node->getNumParents()!=0)
        {
            FindParentVisitor fpv;
            for ( unsigned int i=0; i<node->getNumParents(); ++i )
            {
                node->getParent(i)->accept( fpv );
                if ( fpv.foundWorld )break;
            }
            //this->accept(fpv);
            _parentWorld=fpv.foundWorld;
            btRigidBody *b=getRigidBody();
            if(b)
            {
                if(!dynamic_cast<osgbDynamics::RigidBodyMotionState*>(b->getMotionState()))
                {
///_body is not compatible with osgBullet RigidBody so transform it

                    btMotionState * ms=b->getMotionState();
                    btTransform trans;
                    trans=b->getWorldTransform();

                    osgbDynamics::RigidBodyMotionState *osgms= new osgbDynamics::RigidBodyMotionState();
                    osg::MatrixTransform* mat=dynamic_cast<osg::MatrixTransform*>(node);
                    osgms->setTransform(mat);
                    osgms->setWorldTransform(trans);
                    osgms->setParentTransform(mat->getMatrix());
                    b->setMotionState(osgms);
                    if (ms)delete ms;


                }
                addPhysicalObjectToParentWorld();
            }
        }
    }
    // updatematrix(node,nv);
    //  osg::MatrixTransform::traverse(   nv );


    traverse( node, nv );
}*/
void RigidBody::addPhysicalObjectToParentWorld()
{
    if(_parentWorld)
    {
        btRigidBody*b;
        if(b=getRigidBody())
        {
            if(_parentWorld->getDynamicsWorld()->getCollisionObjectArray().findLinearSearch(b) == _parentWorld->getDynamicsWorld()->getCollisionObjectArray().size())
                _parentWorld->getDynamicsWorld()->addRigidBody(b);
            for(std::vector< osg::ref_ptr<Joint> >::iterator i=_joints.begin(); i!=_joints.end(); i++)
            {
                _parentWorld->addJoint(*i);
            }

            if(_parentWorld->getDebugEnabled())
            {
                _parentWorld-> setDebugEnabled(false);
                _parentWorld-> setDebugEnabled(true);
            }
        }
        else
        {
            OSG_WARN<<"RigidBody: btRigidBody is not setted"<<std::endl;
        }
    }
    else
    {
        OSG_WARN<<"RigidBody: parentworld hasn't been found"<<std::endl;
    }

}

//////////////HELPERS//////////////////////////

btRigidBody* createRigidBody( osgbDynamics::CreationRecord* cr )
{
    osg::Node* root = cr->_sceneGraph;
    if( root == NULL )
    {
        osg::notify( osg::WARN ) << "createRigidBody: CreationRecord has NULL scene graph." << std::endl;
        return( NULL );
    }

    osg::BoundingSphere bs = root->getBound();


    // Bullet collision shapes must be centered on the origin for correct
    // center of mass behavior. Calling code should call
    // CreationRecord::setCenterOfMass() to specify COM. Otherwise, this
    // function uses the bounding volume center as the COM.
    // Translate this subgraph so it is centered on the COM.
    osg::notify( osg::DEBUG_FP ) << "createRigidBody: ";
    osg::Vec3 com;
    if( cr->_comSet )
    {
        // Use user-specified center of mass.
        com = cr->_com;
        osg::notify( osg::DEBUG_FP ) << "User-defined ";
    }
    else
    {
        // Compute from bounding sphere.
        com = bs.center();
        osg::notify( osg::DEBUG_FP ) << "Bounding sphere ";
    }
    osg::notify( osg::DEBUG_FP ) << "center of mass: " << com << std::endl;

    // Create a temporary Transform node containing the center of mass offset and scale vector.
    // Use this as the root of the scene graph for conversion to a collision shape.
    osg::Matrix m( osg::Matrix::translate( -com ) * osg::Matrix::scale( cr->_scale ) );
    osg::ref_ptr< osg::MatrixTransform > tempMtRoot = new osg::MatrixTransform( m );
    tempMtRoot->addChild( root );


    osg::notify( osg::DEBUG_FP ) << "createRigidBody: Creating collision shape." << std::endl;
    btCollisionShape* shape( NULL );
    if( cr->_overall )
    {
        switch( cr->_shapeType )
        {
        case BOX_SHAPE_PROXYTYPE:
            shape = osgbCollision::btCompoundShapeFromBounds( tempMtRoot.get(), BOX_SHAPE_PROXYTYPE );
            break;
        case SPHERE_SHAPE_PROXYTYPE:
            shape = osgbCollision::btCompoundShapeFromBounds( tempMtRoot.get(), SPHERE_SHAPE_PROXYTYPE );
            break;
        case CYLINDER_SHAPE_PROXYTYPE:
            shape = osgbCollision::btCompoundShapeFromBounds( tempMtRoot.get(), CYLINDER_SHAPE_PROXYTYPE, cr->_axis );
            break;
        case TRIANGLE_MESH_SHAPE_PROXYTYPE:
            shape = osgbCollision::btTriMeshCollisionShapeFromOSG( tempMtRoot.get() );
            break;
        case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
        {
            btConvexTriangleMeshShape* cShape = osgbCollision::btConvexTriMeshCollisionShapeFromOSG( tempMtRoot.get() );
            if( cr->_marginSet )
                cShape->setMargin( cr->_margin );
            shape = cShape;
            break;
        }
        case CONVEX_HULL_SHAPE_PROXYTYPE:
        {
            btConvexHullShape* cShape = osgbCollision::btConvexHullCollisionShapeFromOSG( tempMtRoot.get() );
            if( cr->_marginSet )
                cShape->setMargin( cr->_margin );
            shape = cShape;
            break;
        }
        }
    }
    else
    {
        shape = osgbCollision::btCompoundShapeFromOSGGeodes( tempMtRoot.get(),
                cr->_shapeType, cr->_axis, static_cast< unsigned int >( cr->_reductionLevel ) );
    }
    if( shape == NULL )
    {
        osg::notify( osg::WARN ) << "createRigidBody: btCompoundShapeFromOSGGeodes returned NULL." << std::endl;
        return( NULL );
    }

    return( createRigidBody( cr, shape ) );
}

btRigidBody* createRigidBody( osgbDynamics::CreationRecord* cr, btCollisionShape* shape )
{
    osg::Node* root = cr->_sceneGraph;
    if( root == NULL )
    {
        osg::notify( osg::WARN ) << "createRigidBody: CreationRecord has NULL scene graph." << std::endl;
        return( NULL );
    }


    osg::notify( osg::DEBUG_FP ) << "createRigidBody: Creating rigid body." << std::endl;
    btVector3 localInertia( 0, 0, 0 );
    const bool isDynamic = ( cr->_mass != 0.f );
    if( isDynamic )
        shape->calculateLocalInertia( cr->_mass, localInertia );

    // Create MotionState to control OSG subgraph visual reprentation transform
    // from a Bullet world transform. To do this, the MotionState need the address
    // of the Transform node (must be either AbsoluteModelTransform or
    // MatrixTransform), center of mass, scale vector, and the parent (or initial)
    // transform (usually the non-scaled OSG local-to-world matrix obtained from
    // the parent node path).
    osgbDynamics::MotionState* motion = new osgbDynamics::MotionState();
    osg::Transform* trans = dynamic_cast< osg::Transform* >( root );
    if( trans != NULL )        motion->setTransform( trans );

    osg::Vec3 com;
    if( cr->_comSet )
        com = cr->_com;
    else
        com = root->getBound().center();
    motion->setCenterOfMass( com );

    motion->setScale( cr->_scale );
    motion->setParentTransform( cr->_parentTransform );

    // Finally, create rigid body.
    btRigidBody::btRigidBodyConstructionInfo rbInfo( cr->_mass, motion, shape, localInertia );
    rbInfo.m_friction = btScalar( cr->_friction );
    rbInfo.m_restitution = btScalar( cr->_restitution );

#if( BT_BULLET_VERSION > 280 )
    if( cr->_rollingFriction >= 0.f )
        rbInfo.m_rollingFriction = cr->_rollingFriction;
#endif
    if( cr->_linearDamping >= 0.f )
        rbInfo.m_linearDamping = cr->_linearDamping;
    if( cr->_angularDamping >= 0.f )
        rbInfo.m_angularDamping = cr->_angularDamping;

    btRigidBody* rb = new btRigidBody( rbInfo );
    if( rb == NULL )
    {
        osg::notify( osg::WARN ) << "createRigidBody: Created a NULL btRigidBody." << std::endl;
        return( NULL );
    }

    // Last thing to do: Position the rigid body in the world coordinate system. The
    // MotionState has the initial (parent) transform, and also knows how to account
    // for center of mass and scaling. Get the world transform from the MotionState,
    // then set it on the rigid body, which in turn sets the world transform on the
    // MotionState, which in turn transforms the OSG subgraph visual representation.
    btTransform wt;
    motion->getWorldTransform( wt );
    rb->setWorldTransform( wt );

    return( rb );
}






//#include <BulletCollision/CollisionShapes/btHACDCompoundShape.h>

#define VORONOIPOINTS 100
#define CONVEX_MARGIN 0.001
#define BREAKING_THRESHOLD 3
void getVerticesInsidePlanes(const btAlignedObjectArray<btVector3>& planes, btAlignedObjectArray<btVector3>& verticesOut, std::set<int>& planeIndicesOut)
{
    // Based on btGeometryUtil.cpp (Gino van den Bergen / Erwin Coumans)
    verticesOut.resize(0);
    planeIndicesOut.clear();
    const int numPlanes = planes.size();
    int i, j, k, l;
    for (i=0; i<numPlanes; i++)
    {
        const btVector3& N1 = planes[i];
        for (j=i+1; j<numPlanes; j++)
        {
            const btVector3& N2 = planes[j];
            btVector3 n1n2 = N1.cross(N2);
            if (n1n2.length2() > btScalar(0.0001))
            {
                for (k=j+1; k<numPlanes; k++)
                {
                    const btVector3& N3 = planes[k];
                    btVector3 n2n3 = N2.cross(N3);
                    btVector3 n3n1 = N3.cross(N1);
                    if ((n2n3.length2() > btScalar(0.0001)) && (n3n1.length2() > btScalar(0.0001) ))
                    {
                        btScalar quotient = (N1.dot(n2n3));
                        if (btFabs(quotient) > btScalar(0.0001))
                        {
                            btVector3 potentialVertex = (n2n3 * N1[3] + n3n1 * N2[3] + n1n2 * N3[3]) * (btScalar(-1.) / quotient);
                            for (l=0; l<numPlanes; l++)
                            {
                                const btVector3& NP = planes[l];
                                if (btScalar(NP.dot(potentialVertex))+btScalar(NP[3]) > btScalar(0.000001))
                                    break;
                            }
                            if (l == numPlanes)
                            {
                                // vertex (three plane intersection) inside all planes
                                verticesOut.push_back(potentialVertex);
                                planeIndicesOut.insert(i);
                                planeIndicesOut.insert(j);
                                planeIndicesOut.insert(k);
                            }
                        }
                    }
                }
            }
        }
    }
}

static btVector3 curVoronoiPoint;

struct pointCmp
{
    bool operator()(const btVector3& p1, const btVector3& p2) const
    {
        float v1 = (p1-curVoronoiPoint).length2();
        float v2 = (p2-curVoronoiPoint).length2();
        bool result0 = v1 < v2;
        //bool result1 = ((btScalar)(p1-curVoronoiPoint).length2()) < ((btScalar)(p2-curVoronoiPoint).length2());
        //apparently result0 is not always result1, because extended precision used in registered is different from precision when values are stored in memory
        return result0;
    }
};
void voronoiConvexHullShatter(const btAlignedObjectArray<btVector3>& points, const btAlignedObjectArray<btVector3>& verts,
                              const btQuaternion& bbq, const btVector3& bbt, btScalar matDensity,
                              btAlignedObjectArray<btCollisionShape*>&	m_collisionShapes,btDiscreteDynamicsWorld* m_dynamicsWorld, bool getPlanesFromVerticesUsingConvexHullComputer=false)
{


    // points define voronoi cells in world space (avoid duplicates)
    // verts = source (convex hull) mesh vertices in local space
    // bbq & bbt = source (convex hull) mesh quaternion rotation and translation
    // matDensity = Material density for voronoi shard mass calculation
    btConvexHullComputer chc;
    btConvexHullComputer* convexHC = &chc;
    btAlignedObjectArray<btVector3> vertices, chverts;
    btVector3 rbb, nrbb;
    btScalar nlength, maxDistance, distance;
    btAlignedObjectArray<btVector3> sortedVoronoiPoints;
    sortedVoronoiPoints.copyFromArray(points);
    btVector3 normal, plane;
    btAlignedObjectArray<btVector3> planes, convexPlanes;
    std::set<int> planeIndices;
    std::set<int>::iterator planeIndicesIter;
    int numplaneIndices;
    int cellnum = 0;
    int i, j, k, l;

    // Convert verts to world space and get convexPlanes
    int numverts = verts.size();
    chverts.resize(verts.size());
    for (i=0; i < numverts ; i++)
    {
        chverts[i] = quatRotate(bbq, verts[i]) + bbt;
    }
    if (getPlanesFromVerticesUsingConvexHullComputer) btGeometryUtil::getPlaneEquationsFromVertices(chverts, convexPlanes);
    else
    {
        convexHC->compute(&chverts[0].getX(), sizeof(btVector3), numverts, 0.0, 0.0);
        int numFaces = convexHC->faces.size();
        int v0, v1, v2; // vertices
        for (i=0; i < numFaces; i++)
        {
            const btConvexHullComputer::Edge* edge = &convexHC->edges[convexHC->faces[i]];
            v0 = edge->getSourceVertex();
            v1 = edge->getTargetVertex();
            edge = edge->getNextEdgeOfFace();
            v2 = edge->getTargetVertex();
            plane = (convexHC->vertices[v1]-convexHC->vertices[v0]).cross(convexHC->vertices[v2]-convexHC->vertices[v0]).normalize();
            plane[3] = -plane.dot(convexHC->vertices[v0]);
            convexPlanes.push_back(plane);
        }
    }
    const int numconvexPlanes = convexPlanes.size();

    int numpoints = points.size();
    btVector3 curVoronoiPoint;
    // These variables will store the results of the parallel for loop:
    btAlignedObjectArray< btAlignedObjectArray<btVector3> > verticesArray;
    verticesArray.resize(numpoints);
    btAlignedObjectArray<btVector3> curVoronoiPointArray;
    curVoronoiPointArray.resize(numpoints);

    for (i=0; i < numpoints ; i++)
    {
        curVoronoiPoint = curVoronoiPointArray[i] = points[i];
        planes.copyFromArray(convexPlanes);
        for (j=0; j < numconvexPlanes ; j++)
        {
            planes[j][3] += planes[j].dot(curVoronoiPoint);
        }
        maxDistance = SIMD_INFINITY;
        sortedVoronoiPoints.copyFromArray(points);
        //sortedVoronoiPoints.resize(numpoints);
        for (l = 0; l<numpoints; l++)
        {
            //sortedVoronoiPoints[l]=points[l]-curVoronoiPoint;
            sortedVoronoiPoints[l]-=curVoronoiPoint;
        }
        sortedVoronoiPoints.quickSort(pointCmp());
        // No need to undo the subtraction in sortedVoronoiPoints
        for (j=1; j < numpoints; j++)
        {
            normal = sortedVoronoiPoints[j];// - curVoronoiPoint;
            nlength = normal.length();
            if (nlength > maxDistance)
                break;
            plane = normal.normalized();
            plane[3] = -nlength / btScalar(2.);
            planes.push_back(plane);
            getVerticesInsidePlanes(planes, vertices, planeIndices);
            if (vertices.size() == 0)
                break;
            numplaneIndices = planeIndices.size();
            if (numplaneIndices != planes.size())
            {
                planeIndicesIter = planeIndices.begin();
                for (k=0; k < numplaneIndices; k++)
                {
                    if (k != *planeIndicesIter)
                        planes[k] = planes[*planeIndicesIter];
                    planeIndicesIter++;
                }
                planes.resize(numplaneIndices);
            }
            maxDistance = vertices[0].length();
            for (k=1; k < vertices.size(); k++)
            {
                distance = vertices[k].length();
                if (maxDistance < distance)
                    maxDistance = distance;
            }
            maxDistance *= btScalar(2.);
        }
        if (vertices.size() == 0)
            continue;
        verticesArray[i].copyFromArray(vertices);
    }

    for (i=0; i<numpoints; i++)
    {
        const btAlignedObjectArray<btVector3>& vertices = verticesArray[i];
        if (vertices.size()==0) continue;
        const btVector3& curVoronoiPoint = curVoronoiPointArray[i];

        // Clean-up voronoi convex shard vertices and generate edges & faces
        convexHC->compute(&vertices[0].getX(), sizeof(btVector3), vertices.size(),0.0,0.0);

        // At this point we have a complete 3D voronoi shard mesh contained in convexHC

        // Calculate volume and center of mass (Stan Melax volume integration)
        int numFaces = convexHC->faces.size();
        int v0,v1,v2; // Triangle vertices
        btScalar volume = btScalar(0.);
        btVector3 com(0., 0., 0.);
        for (j=0; j < numFaces; j++)
        {
            const btConvexHullComputer::Edge* edge = &convexHC->edges[convexHC->faces[j]];
            v0 = edge->getSourceVertex();
            v1 = edge->getTargetVertex();
            edge = edge->getNextEdgeOfFace();
            v2 = edge->getTargetVertex();
            while (v2 != v0)
            {
                // Counter-clockwise triangulated voronoi shard mesh faces (v0-v1-v2) and edges here...
                btScalar vol = convexHC->vertices[v0].triple(convexHC->vertices[v1], convexHC->vertices[v2]);
                volume += vol;
                com += vol * (convexHC->vertices[v0] + convexHC->vertices[v1] + convexHC->vertices[v2]);
                edge = edge->getNextEdgeOfFace();
                v1 = v2;
                v2 = edge->getTargetVertex();
            }
        }
        com /= volume * btScalar(4.);
        volume /= btScalar(6.);

        // Shift all vertices relative to center of mass
        int numVerts = convexHC->vertices.size();
        for (j=0; j < numVerts; j++)
        {
            convexHC->vertices[j] -= com;
        }

        // Note:
        // At this point convex hulls contained in convexHC should be accurate (line up flush with other pieces, no cracks),
        // ...however Bullet Physics rigid bodies demo visualizations appear to produce some visible cracks.
        // Use the mesh in convexHC for visual display or to perform boolean operations with.

        // Create Bullet Physics rigid body shards
        btCollisionShape* shardShape = new btConvexHullShape(&(convexHC->vertices[0].getX()), convexHC->vertices.size());
        shardShape->setMargin(0.); // for this demo; note convexHC has optional margin parameter for this
        m_collisionShapes.push_back(shardShape);
        btTransform shardTransform;
        shardTransform.setIdentity();
        shardTransform.setOrigin(curVoronoiPoint + com); // Shard's adjusted location
        btDefaultMotionState* shardMotionState = new btDefaultMotionState(shardTransform);
        btScalar shardMass(volume * matDensity);
        btVector3 shardInertia(0.,0.,0.);
        shardShape->calculateLocalInertia(shardMass, shardInertia);
        btRigidBody::btRigidBodyConstructionInfo shardRBInfo(shardMass, shardMotionState, shardShape, shardInertia);
        btRigidBody* shardBody = new btRigidBody(shardRBInfo);
        m_dynamicsWorld->addRigidBody(shardBody);

        cellnum ++;

    }
    printf("Generated %d voronoi btRigidBody shards\n", cellnum);
}
void attachFixedConstraints(btDiscreteDynamicsWorld* m_dynamicsWorld,float breaking_threshold,unsigned int overrideNumSolverIterations,bool useGenericConstraint)
{
    btAlignedObjectArray<btRigidBody*> bodies;

    int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();

    for (int i=0; i<numManifolds; i++)
    {
        btPersistentManifold* manifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        if (!manifold->getNumContacts())
            continue;

        btScalar minDist = 1e30f;
        int minIndex = -1;
        for (int v=0; v<manifold->getNumContacts(); v++)
        {
            if (minDist >manifold->getContactPoint(v).getDistance())
            {
                minDist = manifold->getContactPoint(v).getDistance();
                minIndex = v;
            }
        }
        if (minDist>0.)
            continue;

        btCollisionObject* colObj0 = (btCollisionObject*)manifold->getBody0();
        btCollisionObject* colObj1 = (btCollisionObject*)manifold->getBody1();
        //	int tag0 = (colObj0)->getIslandTag();
//		int tag1 = (colObj1)->getIslandTag();
        btRigidBody* body0 = btRigidBody::upcast(colObj0);
        btRigidBody* body1 = btRigidBody::upcast(colObj1);
        if (bodies.findLinearSearch(body0)==bodies.size())
            bodies.push_back(body0);
        if (bodies.findLinearSearch(body1)==bodies.size())
            bodies.push_back(body1);

        if (body0 && body1)
        {
            if (!colObj0->isStaticOrKinematicObject() && !colObj1->isStaticOrKinematicObject())
            {
                if (body0->checkCollideWithOverride(body1))
                {
                    {
                        btTransform trA,trB;
                        trA.setIdentity();
                        trB.setIdentity();
                        btVector3 contactPosWorld = manifold->getContactPoint(minIndex).m_positionWorldOnA;
                        btTransform globalFrame;
                        globalFrame.setIdentity();
                        globalFrame.setOrigin(contactPosWorld);

                        trA = body0->getWorldTransform().inverse()*globalFrame;
                        trB = body1->getWorldTransform().inverse()*globalFrame;
                        float totalMass = 1.f/body0->getInvMass() + 1.f/body1->getInvMass();


                        if (useGenericConstraint)
                        {
                            btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(*body0,*body1,trA,trB,true);
                            dof6->setOverrideNumSolverIterations(overrideNumSolverIterations);


                            dof6->setBreakingImpulseThreshold(breaking_threshold*totalMass);

                            for (int i=0; i<6; i++)
                                dof6->setLimit(i,0,0);
                            m_dynamicsWorld->addConstraint(dof6,true);

                        }
                        else
                        {
                            btFixedConstraint* fixed = new btFixedConstraint(*body0,*body1,trA,trB);
                            fixed->setBreakingImpulseThreshold(breaking_threshold*totalMass);
                            fixed ->setOverrideNumSolverIterations(overrideNumSolverIterations);
                            m_dynamicsWorld->addConstraint(fixed,true);

                        }

                    }
                }
            }
        }

    }

    /**/for (int i=0; i<bodies.size(); i++)
    {
        m_dynamicsWorld->removeRigidBody(bodies[i]);
        m_dynamicsWorld->addRigidBody(bodies[i]);
    }
}

class MyConvexDecomposition : public ConvexDecomposition::ConvexDecompInterface
{
    std::vector<std::pair<osg::Geometry*,float> >& _convexdecompo;

    btConvexHullComputer* _convexHC ;
public:

    MyConvexDecomposition ( std::vector<std::pair<osg::Geometry*,float> > &convexdecompo)
        :_convexdecompo(convexdecompo)
    {
        _convexHC= new btConvexHullComputer();
    }

    virtual void ConvexDecompResult(ConvexDecomposition::ConvexResult &result)
    {
        osg::Geometry *geom=new osg::Geometry();
        osg::Vec3Array * verts=new osg::Vec3Array();
        verts->resize(result.mHullVcount);
        //  memcpy((void*)verts->getDataPointer(),result.mHullVertices,sizeof(float)*3*result.mHullVcount);
        float *v=result.mHullVertices;
        std::cout<<"ConvexDecompResult result.mHullVcount:"<<result.mHullVcount<<"result.mHullTcount"<<result.mHullTcount<<std::endl;
        for(int i=0; i<result.mHullVcount; i++)
        {
            (*verts)[i]=osg::Vec3(*v,*(v+1),*(v+2));
            v+=3;
        }

        geom->setVertexArray(verts);
        osg::DrawElementsUInt * indices=new osg::DrawElementsUInt(GL_TRIANGLES);
        indices->resize(result.mHullTcount*3);
        // memcpy((void*)indices->getDataPointer(),result.mHullIndices,sizeof(unsigned int)*result.mHullTcount*3);
        unsigned int *ind=result.mHullIndices;
        for(int i=0; i<result.mHullTcount*3; i++)
            (*indices)[i]=*ind++;

        _convexHC->compute(result.mHullVertices, sizeof(float)*3, result.mHullVcount,CONVEX_MARGIN,0.0);
        /*int numFaces = _convexHC->faces.size();
        indices->clear();
        unsigned int j;unsigned int v0, v1, v2;
        for (j=0; j < numFaces; j++) {
        		const btConvexHullComputer::Edge* edge = &_convexHC->edges[_convexHC->faces[j]];
        		v0 = edge->getSourceVertex();
        		v1 = edge->getTargetVertex();
        		edge = edge->getNextEdgeOfFace();
        		v2 = edge->getTargetVertex();
        		indices->push_back(v1);
        		indices->push_back(v2);
        		indices->push_back(v0);
        		}
        		verts->clear();
        for (j=0; j < _convexHC->vertices.size(); j++)
        	verts->push_back(osgbCollision::asOsgVec3(_convexHC->vertices[j]));

            verts->dirty();
        indices->dirty();*/

        geom->addPrimitiveSet(indices);
        /*btConvexHullShape * shape=new btConvexHullShape(&_convexHC->vertices[0].getX(),_convexHC->vertices.size());
        osg::Geode *g=(osg::Geode *)osgbCollision::osgNodeFromBtCollisionShape(shape);


        geom=(osg::Geometry*)g->getDrawable(0);*/





        _convexdecompo.push_back(std::pair<osg::Geometry*,float>(geom,result.mHullVolume));
    }
};


osg::Group*   convexDecomposition(osg::Geometry* g,const ConvexDecompositionParams& params)
{
    osg::ref_ptr<osg::Geometry> geom=g;
    std::vector<std::pair<osg::Geometry*,float> > convexdecompo; ///geom+itsvolume
    bool allareTri=true,hasTristrip=false;
    for(int i=0; i<geom->getNumPrimitiveSets(); i++)
    {
        if(geom->getPrimitiveSet(i)->getMode()!=GL_TRIANGLES)allareTri=false;
        if(geom->getPrimitiveSet(i)->getMode()==GL_TRIANGLE_STRIP)hasTristrip=true;
    }
    osgbCollision::ComputeTriMeshVisitor triv;
    if(hasTristrip)
    {
        osg::ref_ptr<osg::Geode > geode=new osg::Geode();
        geode->addDrawable(geom);
///convert to trianglesoup
        geode->accept(triv);
        geom=new osg::Geometry();
        geom->setVertexArray(triv.getTriMesh());
        std::cout<<triv.getTriMesh()->getNumElements()<<std::endl;
//geom->removePrimitiveSet(0,geom->getNumPrimitiveSets());
//while(geom->getNumPrimitiveSets()>0)geom->removePrimitiveSet(0);
        geom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES,0,triv.getTriMesh()->getNumElements()));
    }

    if(!allareTri||hasTristrip )
    {
        osg::ref_ptr<osg::Geode > geode=new osg::Geode();

        geode->addDrawable(geom);
        ///to convert  to triangles
        osgUtil::Optimizer opt;
        ///convert strip to triangles
        opt.optimize(geode,osgUtil::Optimizer::INDEX_MESH);
    }
    allareTri=true;
    for(int i=0; i<geom->getNumPrimitiveSets(); i++)
        if(geom->getPrimitiveSet(i)->getMode()!=GL_TRIANGLES)allareTri=false;

    if(!allareTri)
    {
        OSG_WARN<<"fractureCollisionShape: geometry cannot be convert to triangles"<<std::endl;
        return 0;
    }
    if(geom->getNumPrimitiveSets()>1)
    {
        OSG_WARN<<"fractureCollisionShape: geometry (even converted) as multiple primitiveset cannot continue"<<std::endl;
        return 0;
    }

    osg::Vec3Array *verts=dynamic_cast<osg::Vec3Array *>(geom->getVertexArray());
    osg::DrawElements * drawelmt=dynamic_cast<osg::DrawElements*>(geom->getPrimitiveSet(0));
    osg::DrawElementsUInt*  indices=dynamic_cast<osg::DrawElementsUInt *>(drawelmt);

    if(!verts)
    {
        OSG_WARN<<"fractureCollisionShape: TODO temp convert vertexarray to Vec3Array"<<std::endl;
        return 0;
    }
    if(!indices)
    {
        ///Convert to osg::DrawElementsUInt*
        indices=new osg::DrawElementsUInt;
        for(int i=0; i<drawelmt->getNumIndices(); i++)
            indices->push_back(drawelmt->getElement(i));
    }

    ConvexDecomposition::DecompDesc desc;

    ConvexBuilder cb(new MyConvexDecomposition(convexdecompo));
    desc.mCallback=&cb;
    unsigned int depth = 5;
    float cpercent     = 5;
    float ppercent     = 15;
    unsigned int maxv  = 16;
    float skinWidth    = 0.0;

    //printf("WavefrontObj num triangles read %i\n",tcount);
    //ConvexDecomposition::DecompDesc desc;

    desc.mVcount       = verts->getNumElements();//wo.mVertexCount;
    desc.mVertices     = (const float*)verts->getDataPointer();//wo.mVertices;
    desc.mTcount       = indices->getNumIndices()/3;//wo.mTriCount;
    desc.mIndices      = (unsigned int *)indices->getDataPointer();

    desc.mDepth        = params.getDepth();
    desc.mCpercent     = params.getConcavityPercentage();
    desc.mPpercent     = params.getVolumeConservationPercent();
    desc.mMaxVertices  = params.getMaxVerticesPerHull();
    desc.mSkinWidth    = params.getSkinWidth();

    cb.process(desc);

    osg::Group * ret=new osg::Group();
    for(std::vector< std::pair<osg::Geometry*,float> >::iterator it=convexdecompo.begin(); it!=convexdecompo.end(); it++)
        ret->addChild(it->first);
    return ret;

}

template<class T>
class myTriangleFunctor : public osg::TriangleFunctor<T>//PrimitiveFunctor, public T
{
    using T::setPoint;
    using T::setCenter;
public:

    myTriangleFunctor( const osg::Vec3 &point=osg::Vec3(),const osg::Vec3 &c=osg::Vec3() ):osg::TriangleFunctor<T>()
    {
        T::setPoint(point);
        T::setCenter(c);
    }
    void setPoint(const osg::Vec3 &point)
    {
        T::setPoint(point);
    }
    void setCenter(const osg::Vec3 &c)
    {
        T::setCenter(c);
    }
};


///test point against dot(normal,point-vertex)
///asume couter clockwized geometry
struct PointInConvexHullFunc
{
    PointInConvexHullFunc():inConvexHull(true) {}
    void setPoint(const osg::Vec3 &point)
    {
        _pt=point;
    }
    void setCenter(const osg::Vec3 &point)
    {
        _center=point;
    }
    void inline operator()( const osg::Vec3 v1, const osg::Vec3 v2, const osg::Vec3 v3, bool _temp )
    {

        if(inConvexHull&&!_temp)
        {
            osg::Vec3 normal=(v2-v1)^(v3-v1);
            if((_pt-v1)*(normal)>=0) inConvexHull=false;
            if((_pt-v2)*(normal)>=0) inConvexHull=false;
            if((_pt-v3)*(normal)>=0) inConvexHull=false;
        }
    }
    bool inConvexHull;
    osg::Vec3 _pt,_center;
};

osg::Group* fractureCollisionShape(osg::Geometry* g,osg::Vec3Array*usersamples,bool useGenericConstraint, bool useMpr )
{
    osg::ref_ptr<osg::Geometry> geom=g;
    std::vector<std::pair<osg::Geometry*,float> > convexdecompo; ///geom+itsvolume
    bool allareTri=true,hasTristrip=false;
    for(int i=0; i<geom->getNumPrimitiveSets(); i++)
    {
        if(geom->getPrimitiveSet(i)->getMode()!=GL_TRIANGLES)allareTri=false;
        if(geom->getPrimitiveSet(i)->getMode()==GL_TRIANGLE_STRIP)hasTristrip=true;
    }
    osgbCollision::ComputeTriMeshVisitor triv;
    if(hasTristrip)
    {
        osg::ref_ptr<osg::Geode > geode=new osg::Geode();
        geode->addDrawable(geom);
///convert to trianglesoup
        geode->accept(triv);
        geom=new osg::Geometry();
        geom->setVertexArray(triv.getTriMesh());
        std::cout<<triv.getTriMesh()->getNumElements()<<std::endl;
//geom->removePrimitiveSet(0,geom->getNumPrimitiveSets());
//while(geom->getNumPrimitiveSets()>0)geom->removePrimitiveSet(0);
        geom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES,0,triv.getTriMesh()->getNumElements()));
    }

    if(!allareTri||hasTristrip )
    {
        osg::ref_ptr<osg::Geode > geode=new osg::Geode();

        geode->addDrawable(geom);
        ///to convert  to triangles
        osgUtil::Optimizer opt;
        ///convert strip to triangles
        opt.optimize(geode,osgUtil::Optimizer::INDEX_MESH);
    }
    allareTri=true;
    for(int i=0; i<geom->getNumPrimitiveSets(); i++)
        if(geom->getPrimitiveSet(i)->getMode()!=GL_TRIANGLES)allareTri=false;

    if(!allareTri)
    {
        OSG_WARN<<"fractureCollisionShape: geometry cannot be convert to triangles"<<std::endl;
        return 0;
    }
    if(geom->getNumPrimitiveSets()>1)
    {
        OSG_WARN<<"fractureCollisionShape: geometry (even converted) as multiple primitiveset cannot continue"<<std::endl;
        return 0;
    }

    osg::Vec3Array *verts=dynamic_cast<osg::Vec3Array *>(geom->getVertexArray());
    osg::DrawElements * drawelmt=dynamic_cast<osg::DrawElements*>(geom->getPrimitiveSet(0));
    osg::ref_ptr<osg::DrawElementsUInt>  indices=dynamic_cast<osg::DrawElementsUInt *>(drawelmt);

    if(!verts)
    {
        OSG_WARN<<"fractureCollisionShape: TODO temp convert vertexarray to Vec3Array"<<std::endl;
        return 0;
    }
    if(!indices)
    {
        ///Convert to osg::DrawElementsUInt*
        indices=new osg::DrawElementsUInt;
        for(int i=0; i<drawelmt->getNumIndices(); i++)
            indices->push_back(drawelmt->getElement(i));
    }

    ConvexDecomposition::DecompDesc desc;

    ConvexBuilder cb(new MyConvexDecomposition(convexdecompo));
    desc.mCallback=&cb;
    unsigned int depth = 7;
    float cpercent     = 1;
    float ppercent     = 99;
    unsigned int maxv  = 32;
    float skinWidth    = 0.0;

    //printf("WavefrontObj num triangles read %i\n",tcount);
    //ConvexDecomposition::DecompDesc desc;

    desc.mVcount       = verts->getNumElements();//wo.mVertexCount;
    desc.mVertices     = (const float*)verts->getDataPointer();//wo.mVertices;
    desc.mTcount       = indices->getNumIndices()/3;//wo.mTriCount;
    desc.mIndices      = (unsigned int *)indices->getDataPointer();

    desc.mDepth        = depth;
    desc.mCpercent     = cpercent;
    desc.mPpercent     = ppercent;
    desc.mMaxVertices  = maxv;
    desc.mSkinWidth    = skinWidth;

    cb.process(desc);


    //////////////////////TEMP WORLD///////////////////////////////////////
    btDiscreteDynamicsWorld* m_dynamicsWorld;//temp world


    ///collision configuration contains default setup for memory, collision setup
    btDefaultCollisionConfiguration *m_collisionConfiguration = new btDefaultCollisionConfiguration();
    //m_collisionConfiguration->setConvexConvexMultipointIterations();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    btCollisionDispatcher *m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);



    if (useMpr)
    {
        printf("using GJK+MPR convex-convex collision detection\n");
        /*btConvexConvexMprAlgorithm::CreateFunc* cf = new btConvexConvexMprAlgorithm::CreateFunc;
        m_dispatcher->registerCollisionCreateFunc(CONVEX_HULL_SHAPE_PROXYTYPE, CONVEX_HULL_SHAPE_PROXYTYPE, cf);
        m_dispatcher->registerCollisionCreateFunc(CONVEX_HULL_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE, cf);
        m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE, CONVEX_HULL_SHAPE_PROXYTYPE, cf);*/
    }
    else
    {
        printf("using default (GJK+EPA) convex-convex collision detection\n");
    }

    btDbvtBroadphase* m_broadphase = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    btSequentialImpulseConstraintSolver* m_solver = new btSequentialImpulseConstraintSolver;


    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
    m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;


    m_dynamicsWorld->setGravity(btVector3(0,-10,0));
    btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
    /////////////////////////////////////////////////////

//btHACDCompoundShape HACD(trianglemess);
    float totalvolume=0;
    ///DEBUG
//convexdecompo .push_back(  std::pair<osg::Geometry*,float>(geom,10000000));
    std::pair<osg::Geometry*,float> biggest=convexdecompo.back();
    for(std::vector< std::pair<osg::Geometry*,float> >::iterator it=convexdecompo.begin(); it!=convexdecompo.end(); it++)
    {
        totalvolume+=(*it).second;
        (*it).first->setInitialBound((*it).first-> computeBoundingBox());

        if(biggest.second<(*it).second)
            biggest=(*it);

        osg::StateSet*ss=(*it).first->getOrCreateStateSet();
        ss->setAttribute(new osg::CullFace());
        ss->setMode(GL_CULL_FACE,osg::StateAttribute::ON);
        ss->setAttribute(new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE ));
        ss->setAttribute(new osg::Point(10));

    }
    ///DEBUG

    /*  std::cout<<convexdecompo.size() <<" convexdecompo"<<biggest.second<< std::endl;
      convexdecompo.clear();
    convexdecompo.push_back(biggest);*/


///ENDDEBUG
myTriangleFunctor< PointInConvexHullFunc > functor;
    for(std::vector< std::pair<osg::Geometry*,float> >::const_iterator it=convexdecompo.begin(); it!=convexdecompo.end(); it++)
    {

        unsigned int numsample=(unsigned int)(           ceil( float(VORONOIPOINTS)*(*it).second/totalvolume)
                               );
                               numsample=1000;
        if(numsample==1)numsample=0;
        osg::Vec3 diff=(*it).first->getInitialBound()._max-(*it).first->getInitialBound()._min;
        btAlignedObjectArray<btVector3> samples;
        btAlignedObjectArray<btVector3> hull;
        osg::Vec3Array* vert=dynamic_cast<osg::Vec3Array*>((*it).first->getVertexArray());
        osg::Vec3f center=(*it).first->getInitialBound().center();
        unsigned int orisize=vert->getNumElements();
        for(unsigned int i=0; i<vert->getNumElements(); i++)
        {
            hull.push_back(osgbCollision::asBtVector3( (*vert)[i]-center));
            // (*vert)[i]=(*vert)[i]-center;
        }
//vert->dirty();
        //pt,center);

        for(osg::Vec3Array::iterator sitr=usersamples->begin();sitr!=usersamples->end()&&samples.size()<numsample;sitr++)
        {

            functor.inConvexHull=true;
            functor.setPoint(*sitr);
            (*it).first->accept( functor );
            if(functor.inConvexHull){
                samples.push_back(osgbCollision::asBtVector3(*sitr));
               sitr=usersamples->erase(sitr);
               if(sitr==usersamples->end())break;
                }
        }
        std::cout<<numsample<<" numsample/ currentsize"<<samples.size()<<std::endl;
       /* while(samples.size()<numsample)
        {
            osg::Vec3 pt(   float(rand() / float(RAND_MAX)) * diff.x() -diff.x()/2.,
                            float(rand() / float(RAND_MAX)) * diff.y() -diff.y()/2.,
                            float(rand() / float(RAND_MAX)) * diff.z() -diff.z()/2.);
            pt+=center;

            functor.inConvexHull=true;
            functor.setPoint(pt);

            (*it).first->accept( functor );
            if(functor.inConvexHull)
            {
                samples.push_back(osgbCollision::asBtVector3(pt));
                ///DEBUG
                vert->push_back(pt);
            }
        }*/

        btScalar matDensity = 1;
        if(samples.size()>1)
        {///DEBUG
             (*it).first->addPrimitiveSet(new osg::DrawArrays(GL_POINTS,orisize,vert->getNumElements()-1-orisize));

            btQuaternion bbq(0,0,0,1);
            btVector3 bbt=asBtVector3(center);
            bbq.normalize();
            //for(unsigned int i=0; i<samples.size(); i++)            hull.push_back(samples[i]-asBtVector3(center));
            voronoiConvexHullShatter(samples,hull,bbq,bbt,matDensity,m_collisionShapes,m_dynamicsWorld);
        }else{
        ///add a single rigid
        osg::Matrix m;m.makeTranslate(-center);
        osg::ref_ptr<osg::MatrixTransform> mat=new osg::MatrixTransform(m);
        osg::ref_ptr<osg::Geode> g=new osg::Geode();mat->addChild(g);
        g->addDrawable((*it).first);
            btCollisionShape* shardShape =osgbCollision::btConvexHullCollisionShapeFromOSG( mat);
            shardShape->setMargin(0.); // for this demo; note convexHC has optional margin parameter for this
            m_collisionShapes.push_back(shardShape);
            btTransform shardTransform;
            shardTransform.setIdentity();
        shardTransform.setOrigin(asBtVector3(center)); // Shard's adjusted location
            btDefaultMotionState* shardMotionState = new btDefaultMotionState(shardTransform);
            btScalar shardMass((*it).second * matDensity);
            btVector3 shardInertia(0.,0.,0.);
            shardShape->calculateLocalInertia(shardMass, shardInertia);
            btRigidBody::btRigidBodyConstructionInfo shardRBInfo(shardMass, shardMotionState, shardShape, shardInertia);
            btRigidBody* shardBody = new btRigidBody(shardRBInfo);
            m_dynamicsWorld->addRigidBody(shardBody);
        }

    }
    printf("useGenericConstraint = %d\n", useGenericConstraint);





    for (int i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ; i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        obj->getCollisionShape()->setMargin(CONVEX_MARGIN+0.01);
    }
    m_dynamicsWorld->performDiscreteCollisionDetection();

    for (int i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ; i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        obj->getCollisionShape()->setMargin(CONVEX_MARGIN);
    }

    attachFixedConstraints(m_dynamicsWorld,BREAKING_THRESHOLD,30,useGenericConstraint);

    ///TODO bake world dynamics and constraints
    osg::Group *fractured=new osg::Group;


    std::map<btRigidBody*,RigidBody*> rigs;

    for (int i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ; i--)
    {
        btRigidBody * col=dynamic_cast<btRigidBody * >(m_dynamicsWorld->getCollisionObjectArray()[i]);
        if(col)
        {

            btConvexHullShape * collision=dynamic_cast<btConvexHullShape * >(col->getCollisionShape());
            if(collision)
            {
                osg::Node *n=osgbCollision::osgNodeFromBtCollisionShape(collision);//,col->getWorldTransform());
                osgbDynamics::RigidBody* rig=new RigidBody();
                rig->setRigidBody(col);



                osg::MatrixTransform * rignode=new osg::MatrixTransform();
                rignode->addUpdateCallback(rig);
                rignode->addChild(n);
                rigs[rig->getRigidBody()]=rig;

                fractured->addChild(rignode);
            }
            else
            {
                OSG_WARN<<"warning fracturation : cant convert shape to btConvexHull"<<std::endl;
            }
        }
        else
        {
            OSG_WARN<<"warning fracturation : cant convert collision object to rigidbody"<<std::endl;
        }
    }




    for(int j=0; j<m_dynamicsWorld->getNumConstraints(); j++)
    {
        btTypedConstraint * constraint=m_dynamicsWorld->getConstraint(j);
        //std::vector<RigidBody*>::iterator itrA=std::find(rigs.begin(),rigs.end(),&constraint->getRigidBodyA(),compareRigs);
        //std::vector<RigidBody*>::iterator itrB=std::find(rigs.begin(),rigs.end(),&constraint->getRigidBodyB(),compareRigs);
        std::map<btRigidBody*,RigidBody*>::iterator itrA=rigs.find(&constraint->getRigidBodyA());
        std::map<btRigidBody*,RigidBody*>::iterator itrB=rigs.find(&constraint->getRigidBodyB());
        if(itrA!=rigs.end()&&itrB!=rigs.end())
        {
            Joint *joint=new Joint();
            joint->setBodyA(itrA->second);
            joint->setBodyB(itrB->second);
            joint->setConstraint(constraint);
            (itrA->second)->addJoint(joint);
            (itrB->second)->addJoint(joint);

        }
        else
        {
            OSG_WARN<<"warning fracturation : constraint rigs not found"<<std::endl;
        }
    }



    ///DEBUG
    /*osg::Geode*geode=new osg::Geode();
    for(std::vector<std::pair<osg::Geometry*,float> >::iterator it=convexdecompo.begin(); it!=convexdecompo.end(); it++)
        geode->addDrawable(it->first);
    fractured->addChild(geode);*/


    osgUtil::SmoothingVisitor sv;
    sv.setCreaseAngle(0);
    fractured->accept(sv);

    return fractured;
}

///HLEPERVISITORS/////////////////////////////////////////////

void CreateRigidFromSkeletonVisitor::apply(osg::Geode&g)
{
    osg::Matrix subMatrix2 = computeLocalToWorld( g.getParentalNodePaths()[0] );
    for(int i=0; i<g.getNumDrawables(); i++)
    {
    if( dynamic_cast<osgAnimation::RigGeometry*>(g.getDrawable(i))){
        _collecteddrawables.push_back( std::pair<osg::Geometry*,osg::Matrix> (g.getDrawable(i)->asGeometry(),subMatrix2));
        ComputeVolumeFunctor volfunc;
        _collecteddrawables.back().first->accept(volfunc);
        _totalvolume+=volfunc.getComputedVolume();
        }
    }
    traverse(g);
}
void CreateRigidFromSkeletonVisitor::computeRig(){
std::map<osgAnimation::Bone*,std::vector<osgAnimation::RigGeometry*> >bonemap;
  for(std::vector<std::pair<osg::Geometry*,osg::Matrix> > ::iterator itdr=_collecteddrawables.begin(); itdr!=_collecteddrawables.end(); itdr++)
    {

        osgAnimation::RigGeometry* geom=dynamic_cast< osgAnimation::RigGeometry*>(itdr->first);
        if(geom)
        {

    osgAnimation::BoneMapVisitor mapVisitor;
    geom->getSkeleton()->accept(mapVisitor);
    osgAnimation::BoneMap bm = mapVisitor.getBoneMap();
     osgAnimation::Bone *maxbone=0;
     float maxboneweight=-1;
    //initVertexSetFromBones(bm, geom.getVertexInfluenceSet().getUniqVertexSetToBoneSetList());
    {
     std::vector<osgAnimation::VertexInfluenceSet::UniqVertexSetToBoneSet> influence=geom->getVertexInfluenceSet().getUniqVertexSetToBoneSetList();
    int size = influence.size();
  //  _boneSetVertexSet.resize(size);
    for (int i = 0; i < size; i++)
    {
        const  osgAnimation::VertexInfluenceSet::UniqVertexSetToBoneSet& inf = influence[i];
        int nbBones = inf.getBones().size();
        // osgAnimation::BoneWeightList& boneList = _boneSetVertexSet[i].getBones();

        double sumOfWeight = 0;
        for (int b = 0; b < nbBones; b++)
        {
            const std::string& bname = inf.getBones()[b].getBoneName();
            float weight = inf.getBones()[b].getWeight();
            osgAnimation:: BoneMap::const_iterator it = bm.find(bname);
            if (it == bm.end() )
            {
               /* if (_invalidInfluence.find(bname) != _invalidInfluence.end()) {
                    _invalidInfluence[bname] = true;
                    OSG_WARN << "RigTransformSoftware Bone " << bname << " not found, skip the influence group " <<bname  << std::endl;
                }*/
                continue;
            }
           // Bone* bone = it->second.get();
            if( maxboneweight<  weight){
            maxboneweight= weight;
            maxbone=it->second.get();
            }
           // boneList.push_back( osgAnimation::BoneWeight(bone, weight));
           // sumOfWeight += weight;
        }
    }
    }
    bonemap[maxbone].push_back(geom);




    }
}

for(std::map<osgAnimation::Bone*,std::vector<osgAnimation::RigGeometry*> >::iterator it=bonemap.begin();it!=bonemap.end();it++){
osg::ref_ptr<osg::Geode> ge=new osg::Geode;
            ComputeCenterOfMassFunctor comfunc;
for(std::vector<osgAnimation::RigGeometry*>::iterator rigit=it->second.begin();rigit!=it->second.end();rigit++){
ge->addDrawable((*rigit)->getSourceGeometry());
 (*rigit)->getSourceGeometry()->accept(comfunc);
}
   btConvexHullShape *shape=osgbCollision::btConvexHullCollisionShapeFromOSG(ge);
            ///DEBUG
            //shape->setMargin(0.0);
            osg::MatrixTransform *matrans=it->first;
            osg::Geode *geode=new osg::Geode;
           // geode->addDrawable(geom);
            RigidBody* rig=new RigidBody();
            //ig->setName(geom->getName());

          //  geom->accept(comfunc);
            float frac=comfunc.getComputedVolume()/_totalvolume;

            osg::ref_ptr<CreationRecord> cr=new CreationRecord(*_overallcr.get());
            cr->_mass*=frac;
            matrans->setMatrix( computeLocalToWorld(it->first->getParentalNodePaths()[0] ));
            cr->_sceneGraph=matrans;
            cr->_parentTransform=computeLocalToWorld(it->first->getParentalNodePaths()[0] );

            cr->setCenterOfMass(osg::Vec3());
            // cr->setCenterOfMass(comfunc.getComputedCOM());
            OSG_WARN<<"comfunc.getComputedCOM()"<<comfunc.getComputedCOM()<<std::endl;
            OSG_WARN<<"comfunc.getComputedVolume()"<<frac/*comfunc.getComputedVolume()*/<<std::endl;
            //matrans->addChild(geode);
  btRigidBody * btrig=osgbDynamics::createRigidBody(cr,shape);
            rig->setRigidBody(btrig);
//btrig->forceActivationState(DISABLE_SIMULATION);
            matrans->addUpdateCallback(rig);
            _result->addChild(matrans);

}
}


void CreateRigidVisitor::apply(osg::Geode&g)
{
    osg::Matrix subMatrix2 = computeLocalToWorld( g.getParentalNodePaths()[0] );
    for(int i=0; i<g.getNumDrawables(); i++)
    {
        _collecteddrawables.push_back( std::pair<osg::Drawable*,osg::Matrix> (g.getDrawable(i),subMatrix2));
        ComputeVolumeFunctor volfunc;
        _collecteddrawables.back().first->accept(volfunc);
        _totalvolume+=volfunc.getComputedVolume();
    }

    traverse(g);
}

osg::Group * CreateRigidVisitor::getResult()
{
    _result->removeChildren(0,_result->getNumChildren());
    for(std::vector<std::pair<osg::Drawable*,osg::Matrix> > ::iterator itdr=_collecteddrawables.begin(); itdr!=_collecteddrawables.end(); itdr++)
    {

        osg::Geometry* geom=dynamic_cast<osg:: Geometry*>(itdr->first);
        if(geom)
        {


///TODO test concavity
            btConvexHullShape *shape=osgbCollision::btConvexHullCollisionShapeFromOSG(geom);
            ///DEBUG
            //shape->setMargin(0.0);
            osg::MatrixTransform *matrans=new osg::MatrixTransform();
            osg::Geode *geode=new osg::Geode;
            geode->addDrawable(geom);
            RigidBody* rig=new RigidBody();
            rig->setName(geom->getName());

            ComputeCenterOfMassFunctor comfunc;
            geom->accept(comfunc);
            float frac=comfunc.getComputedVolume()/_totalvolume;

            osg::ref_ptr<CreationRecord> cr=new CreationRecord(*_overallcr.get());
            cr->_mass*=frac;
            matrans->setMatrix(itdr->second);
            cr->_sceneGraph=matrans;
            cr->_parentTransform=itdr->second;
#if 1
            cr->setCenterOfMass(osg::Vec3());
            // cr->setCenterOfMass(comfunc.getComputedCOM());
            OSG_WARN<<"comfunc.getComputedCOM()"<<comfunc.getComputedCOM()<<std::endl;
            OSG_WARN<<"comfunc.getComputedVolume()"<<frac/*comfunc.getComputedVolume()*/<<std::endl;
            matrans->addChild(geode);
#else
///recenter geometry
///TODO perhaps a copy of geometry would avoid input hacking
            osgUtil::Optimizer opt;
            osg::ref_ptr<osg::Group> tmpgr=new osg::Group();
            osg::ref_ptr<osg::MatrixTransform >tmpmat=new osg::MatrixTransform();
            tmpgr->addChild(tmpmat);
            tmpmat->addChild(geode);
            osg::Matrix m;
            m.makeTranslate(comfunc.getComputedCOM());
            tmpmat->setMatrix(m);
            opt.optimize(tmpgr,osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS);
            matrans->addChild(geode);
#endif
            btRigidBody * btrig=osgbDynamics::createRigidBody(cr,shape);
            rig->setRigidBody(btrig);
//btrig->forceActivationState(DISABLE_SIMULATION);
            matrans->addUpdateCallback(rig);
            _result->addChild(matrans);


        }
        else
        {
///TODO ShapeDrawable test

        }



    }
    return _result;
}

void CreateRigidVisitor::apply(osg::Group&g)
{


    osg::NodeVisitor::apply(g);
}
AttachRigidVisitor::~AttachRigidVisitor()
{
///TODO destroy temp world
    delete m_dynamicsWorld;
}
AttachRigidVisitor::AttachRigidVisitor():osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{
    _breakthreshold=5;
    _useGenericConstraint=false;
//////////////////////TEMP WORLD///////////////////////////////////////
    //btDiscreteDynamicsWorld* m_dynamicsWorld;//temp world


    ///collision configuration contains default setup for memory, collision setup
    btDefaultCollisionConfiguration *m_collisionConfiguration = new btDefaultCollisionConfiguration();
    //m_collisionConfiguration->setConvexConvexMultipointIterations();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    btCollisionDispatcher *m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);



    if (false)//useMpr)
    {
        printf("using GJK+MPR convex-convex collision detection\n");
        /*btConvexConvexMprAlgorithm::CreateFunc* cf = new btConvexConvexMprAlgorithm::CreateFunc;
        m_dispatcher->registerCollisionCreateFunc(CONVEX_HULL_SHAPE_PROXYTYPE, CONVEX_HULL_SHAPE_PROXYTYPE, cf);
        m_dispatcher->registerCollisionCreateFunc(CONVEX_HULL_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE, cf);
        m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE, CONVEX_HULL_SHAPE_PROXYTYPE, cf);*/
    }
    else
    {
        printf("using default (GJK+EPA) convex-convex collision detection\n");
    }

    btDbvtBroadphase* m_broadphase = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    btSequentialImpulseConstraintSolver* m_solver = new btSequentialImpulseConstraintSolver;


    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
    m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;


    m_dynamicsWorld->setGravity(btVector3(0,-10,0));
    // btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
    //
}
void AttachRigidVisitor::apply( osg::MatrixTransform& node )
{
    RigidBody* rig;
    if(rig=dynamic_cast<RigidBody*>(node.getUpdateCallback()))
    {
        btRigidBody* btrig=rig->getRigidBody();
        if(btrig)
        {
            rigs[btrig]=rig;
            m_dynamicsWorld->addRigidBody(btrig);

        }
        else {}
    }
    traverse( node );
}
// osgbDynamics
void AttachRigidVisitor::generateConstraints()
{

    for (int i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ; i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        obj->getCollisionShape()->setMargin(CONVEX_MARGIN+0.01);
    }
    m_dynamicsWorld->performDiscreteCollisionDetection();

    for (int i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ; i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        obj->getCollisionShape()->setMargin(CONVEX_MARGIN);


    }

    attachFixedConstraints(m_dynamicsWorld,_breakthreshold,30,_useGenericConstraint);
    for(int j=0; j<m_dynamicsWorld->getNumConstraints(); j++)
    {
        btTypedConstraint * constraint=m_dynamicsWorld->getConstraint(j);
        //std::vector<RigidBody*>::iterator itrA=std::find(rigs.begin(),rigs.end(),&constraint->getRigidBodyA(),compareRigs);
        //std::vector<RigidBody*>::iterator itrB=std::find(rigs.begin(),rigs.end(),&constraint->getRigidBodyB(),compareRigs);
        std::map<btRigidBody*,RigidBody*>::iterator itrA=rigs.find(&constraint->getRigidBodyA());
        std::map<btRigidBody*,RigidBody*>::iterator itrB=rigs.find(&constraint->getRigidBodyB());
        if(itrA!=rigs.end()&&itrB!=rigs.end())
        {
            Joint *joint=new Joint();
            joint->setBodyA(itrA->second);
            joint->setBodyB(itrB->second);
            joint->setConstraint(constraint);
            (itrA->second)->addJoint(joint);
            (itrB->second)->addJoint(joint);

        }
        else
        {
            OSG_WARN<<"warning attachFixedConstraints : constraint rigs not found"<<std::endl;
        }
    }
}
}
