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

class FindParentVisitor : public osg::NodeVisitor
{
public:
    FindParentVisitor()
        : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_PARENTS), foundWorld(NULL)  {}

    void apply( osg::Node& node )
    {

        if( !foundWorld )
            foundWorld = dynamic_cast<World*>(&node);
        /*osg::Callback* cb = node.getUpdateCallback();
        while ( cb && !foundWorld )
        {
            foundWorld = dynamic_cast<World*>(cb);

            cb = cb->getNestedCallback();
        }*/
        traverse( node );
    }

    World* foundWorld;
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
    for(std::vector< Joint* >::iterator i=_joints.begin(); i!=_joints.end(); i++)
        if(*i ==p)return;

    _joints.push_back(p);
    if(_parentWorld)_parentWorld->addJoint(p);
}
void RigidBody::removeJoint(Joint*p)
{
    for(std::vector< Joint* >::iterator i=_joints.begin(); i!=_joints.end(); i++)
    {
        if(*i ==p)
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
        FindParentVisitor fpv;
        node->accept(fpv);
        _parentWorld=fpv.foundWorld;
        //addPhysicalObjectToParentWorld(static_cast<osg::MatrixTransform*>(node));
         if(_body){
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
            _parentWorld->getDynamicsWorld()->addRigidBody(_body);
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
            for(std::vector< Joint* >::iterator i=_joints.begin(); i!=_joints.end(); i++)
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
    if( trans != NULL )
        motion->setTransform( trans );

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


// osgbDynamics
}
