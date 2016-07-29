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

/*#include <osgbDynamics/MotionState.h>*/
#include <osgbCollision/GLDebugDrawer.h>

#include <osg/NodeVisitor>
#include <osg/Notify>

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>

#ifndef USE_REFERENCE_TIME
#   define USE_REFERENCE_TIME DBL_MAX
#endif

using namespace osgbDynamics;
//osg::ref_ptr<osg::Geode> World::_debugdrawable=0;
World::World()
    : _debugdraw(false),  _btworld(NULL),_deltaTime( USE_REFERENCE_TIME)
{
    setWorldType(RIGID_ONLY);
    setDataVariance(osg::Object::DYNAMIC);
}

World::~World()
{
}

World::World( const World& copy, const osg::CopyOp& copyop )
    :   osg::Group(copy, copyop)
{

}
void World::setWorldType(WorldType t)
{


    if(_worldtype!=t || !_btworld)
    {
        _worldtype=t;

///init world
        // btDefaultCollisionConfiguration * collisionConfiguration =0;
        btCollisionDispatcher * dispatcher = 0;
        btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

        btVector3 worldAabbMin( -10000, -10000, -10000 );
        btVector3 worldAabbMax( 10000, 10000, 10000 );
        btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

        // btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );







        /*

        btVector3 gravity( 0, 0, -10.17 );
        _btworld->setGravity( gravity );*/






        switch(_worldtype)
        {
//case RIGID_ONLY: _btworld=new btDiscreteDynamicsWorld();break;
        case RIGID_AND_SOFT:
        {
            btSoftBodyRigidBodyCollisionConfiguration* collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
            dispatcher = new btCollisionDispatcher( collisionConfiguration );
            _btworld=new btSoftRigidDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );
            //   _btworld->getWorldInfo().m_gravity = gravity;
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().m_broadphase = inter;
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().air_density = btScalar( 1.2 );
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().water_density = 0;
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().water_offset = 0;
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().water_normal = btVector3( 0, 0, 0 );
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().m_sparsesdf.Initialize();
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().m_dispatcher = dispatcher;
        }
        break;
        default:
        {
            btDefaultCollisionConfiguration*  collisionConfiguration = new btDefaultCollisionConfiguration();
            dispatcher = new btCollisionDispatcher( collisionConfiguration );
            _btworld=new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );
        }
        }

        _btworld->setGravity( btVector3( 0, 0, -10 ) );
    }

}
/*
void World::update( osg::Node* node, osg::NodeVisitor* nv )
{
    const osg::FrameStamp* fs = nv->getFrameStamp();
    if ( _deltaTime==USE_REFERENCE_TIME )
        _btworld->stepSimulation( fs->getReferenceTime() - _prevousReferenceTime);
    else
        _btworld->stepSimulation( _deltaTime );
    _prevousReferenceTime = fs->getReferenceTime();

}*/


class  DebugDrawable:public osg::Geometry
{
public:
    DebugDrawable() {}
    DebugDrawable(const DebugDrawable&x,osg::CopyOp op=osg::CopyOp::SHALLOW_COPY) {}
    META_Node(osgbDynamics,DebugDrawable)
    DebugDrawable(osgbCollision::GLDebugDrawer *d,btDiscreteDynamicsWorld*w ):dbgDraw(d),dynamicsWorld(w) {}
    virtual void drawImplementation(osg::RenderInfo& /*renderInfo*/) const
    {
        dbgDraw->BeginDraw();

        dynamicsWorld->debugDrawWorld();
        dbgDraw->EndDraw();

    }
    /* virtual void traverse(osg::NodeVisitor&nv){
     Drawable::traverse(nv);
     }*/
protected:
    osgbCollision::GLDebugDrawer *dbgDraw;
    btDiscreteDynamicsWorld*dynamicsWorld;

};

void World::addJoint(Joint*j)
{
///avoid duplicate
    for(std::vector<osg::ref_ptr<Joint> >::iterator i=_joints.begin(); i!=_joints.end(); i++)
        if(i->get()==j)return;

    if(j->getConstraint())
    {
        if(_btworld)
        {
            _joints.push_back(j);
            _btworld->addConstraint(j->getConstraint());


            OSG_WARN<<"joint constraint added"<<std::endl;
        }
        else
        {
            _joints2add.push_back(j);
        }
        if(j->getBodyA())j->getBodyA()->addJoint(j);
        if(j->getBodyB())j->getBodyB()->addJoint(j);
    }
    else
    {
///joint constraint not defined yet
//debug
        OSG_WARN<<"Warning joint constraint not defined yet"<<std::endl;
    }
}
void World::removeJoint(Joint*j)
{
    for(std::vector<osg::ref_ptr<Joint> >::iterator i=_joints.begin(); i!=_joints.end(); i++)
    {
        if(i->get()==j)
        {
            _btworld->removeConstraint(j->getConstraint());
            osg::ref_ptr<Joint> ptrj=j;
            _joints.erase(i);
            if(j->getBodyA())j->getBodyA()->removeJoint(j);
            if(j->getBodyB())j->getBodyB()->removeJoint(j);

            return;
        }
    }
}

void World::setDebugEnabled(bool b)
{
    if (_debugdraw==b)return;
    _debugdraw=b;
    if(_debugdraw)
    {
        dbgDraw = new osgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        _btworld->setDebugDrawer( dbgDraw );
        _debugdrawable=new osg::Geode();
        _debugdrawable->addDrawable(new DebugDrawable (dbgDraw,_btworld));


    }
    else
    {

        if(_btworld->getDebugDrawer())
        {
            delete _btworld->getDebugDrawer();
            _btworld->setDebugDrawer( 0);

        }
    }

}
void World::traverse(osg::NodeVisitor &nv)
{
#define DEBUGSTUFF  if(_debugdraw)\
        {\
        dbgDraw->getSceneGraph()->traverse(nv);\
        _debugdrawable->traverse(nv);\
            osg::Group::traverse(nv);\
        }\
        else    osg::Group::traverse(nv);

    ///
    switch(nv.getVisitorType())
    {

    case osg::NodeVisitor::UPDATE_VISITOR:
        ///update the world
    {
        while(!_joints2add.empty())
        {
            addJoint(_joints2add.back());
            _joints2add.pop_back();
        }


        const osg::FrameStamp* fs = nv.getFrameStamp();
        if ( _deltaTime==USE_REFERENCE_TIME )
            _btworld->stepSimulation( fs->getReferenceTime() - _prevousReferenceTime);
        else
            _btworld->stepSimulation( _deltaTime );
        _prevousReferenceTime = fs->getReferenceTime();
        osg::Group::traverse(nv);
    }
    break;
    case osg::NodeVisitor::CULL_VISITOR:
        DEBUGSTUFF
        break;
    default:
        osg::Group::traverse(nv);
    }

}


