/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2011 Robert
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/
// Written by J.Valentin

#include <osgbDynamics/World.h>

/*#include <osgbDynamics/MotionState.h>
#include <osgbCollision/Utils.h>

#include <osg/Object>
#include <osg/Notify>
*/
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>

#ifndef USE_REFERENCE_TIME
#   define USE_REFERENCE_TIME DBL_MAX
#endif

using namespace osgbDynamics;

World::World()
:   _btworld(NULL),_worldtype(RIGID_ONLY),_deltaTime( USE_REFERENCE_TIME)
{
}

World::~World()
{
}

World::World( const World& copy, const osg::CopyOp& copyop )
:   osg::NodeCallback(copy, copyop)
{

}
 void World::setWorldType(WorldType t){


 if(_worldtype!=t || !_btworld){
 _worldtype=t;

 ///init world
     btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

   // btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );





 switch(_worldtype){
 //case RIGID_ONLY: _btworld=new btDiscreteDynamicsWorld();break;
 case RIGID_AND_SOFT:_btworld=new btSoftRigidDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );break;
 default: _btworld=new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

 }

    _btworld->setGravity( btVector3( 0, 0, -10 ) );
 }

 }

void World::update( osg::Node* node, osg::NodeVisitor* nv ){
     const osg::FrameStamp* fs = nv->getFrameStamp();
        if ( _deltaTime==USE_REFERENCE_TIME )
      _btworld->stepSimulation( fs->getReferenceTime() - _prevousReferenceTime);
        else
           _btworld->stepSimulation( _deltaTime );
        _prevousReferenceTime = fs->getReferenceTime();

     }
void World::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
  /*  if ( _active )
    {
        if ( !_parent && !_initParentElement )
        {
            AbstractInterface* interface = engineInstance();
            if ( interface )
            {
                AbstractInterface::ElementAndNode ean = interface->findParentElement(node);
                _parent = ean.first; _parentNode = ean.second;
                _initParentElement = true;
            }
        }

        if ( _dirtyParts>0 )
        {
            reallocate( node, nv, (_dirtyParts&RECREATE_ELEMENT)>0 );
            _dirtyParts = 0;
        }
        else
            update( node, nv );
    }
    traverse( node, nv );

    if ( _active )
    {
        postevent( node, nv );
    }*/update( node, nv );
    traverse( node, nv );
}
