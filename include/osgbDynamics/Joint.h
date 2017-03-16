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

#ifndef __OSGBDYNAMICS_Joint_H__
#define __OSGBDYNAMICS_Joint_H__  1

#include <osgbDynamics/Export.h>
#include <osg/Object>
#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>

namespace osgbDynamics
{
class World;
class RigidBody;

class OSGBDYNAMICS_EXPORT Joint : public  osg::Object
{
public:
    Joint();
    Joint( const Joint& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,Joint)

    const btTypedConstraint* getConstraint()const;
    btTypedConstraint* getConstraint();

    void setConstraint( btTypedConstraint *b );
    const osgbDynamics::RigidBody *getBodyA()const    {        return _rigA;    }
     osgbDynamics::RigidBody *getBodyA()    {        return _rigA;    }
    void setBodyA(RigidBody *c);
    const RigidBody *getBodyB()const    {        return _rigB;    }
     RigidBody *getBodyB()    {        return _rigB;    }
    void setBodyB(RigidBody *c);

protected:
    ~Joint();

    RigidBody* _rigA;
    RigidBody* _rigB;
    btTypedConstraint*_btConstraint;

};

// osgbDynamics
}


#endif
