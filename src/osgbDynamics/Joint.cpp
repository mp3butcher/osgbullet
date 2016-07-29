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

#include <osgbDynamics/Joint.h>
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

Joint::Joint():_rigA(0),_rigB(0){}
Joint::~Joint(){}

    Joint::Joint( const Joint& copy, const osg::CopyOp& copyop ){}
const btTypedConstraint* Joint::getConstraint()const
{
    const   osgbCollision::RefBulletObject<  btTypedConstraint >*  refptr=
        dynamic_cast<  const   osgbCollision::RefBulletObject<  btTypedConstraint >* >( getUserData() );

    return (refptr?refptr->get():0);
}

btTypedConstraint* Joint::getConstraint()
{
    osgbCollision::RefBulletObject<  btTypedConstraint >*  refptr=
        dynamic_cast<     osgbCollision::RefBulletObject<  btTypedConstraint >* >( getUserData() );

    return (refptr?refptr->get():0);
}

void Joint::setConstraint( btTypedConstraint *b )
{
    setUserData(new osgbCollision::RefBulletObject<btTypedConstraint>(b));
    }
  void Joint::setBodyA(RigidBody *c)
    {
    if(_rigA==c)return;
    if(_rigA!=c && _rigA)_rigA->removeJoint(this);
        _rigA=c;
    if(_rigA)_rigA->addJoint(this);
    }

    void Joint::setBodyB(RigidBody *c)
    {
    if(_rigB==c)return;
    if(_rigB!=c && _rigB)_rigB->removeJoint(this);
        _rigB=c;
    if(_rigB)_rigB->addJoint(this);
    }

// osgbDynamics
}
