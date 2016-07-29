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

#ifndef __OSGBDYNAMICS_SOFT_BODY_H__
#define __OSGBDYNAMICS_SOFT_BODY_H__ 1

#include <osgbDynamics/Export.h>
#include <osg/NodeCallback>
#include <osgbDynamics/CreationRecord.h>
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>

#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osgbCollision/AbsoluteModelTransform.h>
#include <osgbCollision/RefBulletObject.h>

namespace osgbDynamics
{
/** use to filter collisions  **/
class World;
class RigidBody;
class SoftBody;
/*
class OSGBDYNAMICS_EXPORT PhysicalObject : public osg::NodeCallback
{
public:
    PhysicalObject():_parentWorld(0) {};
    PhysicalObject( const PhysicalObject& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
META_Object(osgbDynamics,PhysicalObject)


    virtual RigidBody * asRigidBody()
    {
        return 0;
    }
    virtual SoftBody * asSoftBody()
    {
        return 0;
    }

    void operator()( osg::Node* node, osg::NodeVisitor* nv );


protected:

    virtual void updatematrix( osg::Node* node, osg::NodeVisitor* nv )=0;
    virtual void addPhysicalObjectToParentWorld()=0;
    ~PhysicalObject();
    World * _parentWorld;

};*/


class OSGBDYNAMICS_EXPORT SoftBody : public osg::Geometry
{
public:
    SoftBody();
    SoftBody( const SoftBody& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
#if 1
    virtual osg::Object* cloneType() const { return new SoftBody (); }
        virtual osg::Object* clone(const osg::CopyOp& copyop) const { return new SoftBody (*this,copyop); }
        virtual bool isSameKindAs(const osg::Object* obj) const { return dynamic_cast<const SoftBody *>(obj)!=NULL; }
        virtual const char* className() const { return "SoftBody"; }
        virtual const char* libraryName() const { return "osgbDynamics"; }
        virtual void accept(osg::NodeVisitor& nv) { if (nv.validNodeMask(*this)) { nv.pushOntoNodePath(this);
        //nv.apply(*this);
        ///forcedupdatetraversal
        if(nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR)traverse(nv);
        else nv.apply(*this);
         //if (nv.getTraversalMode()==osg::NodeVisitor::TRAVERSE_PARENTS) nv.apply(*this);
           // else if (nv.getTraversalMode()!=osg::NodeVisitor::TRAVERSE_NONE) traverse(nv);
        nv.popFromNodePath();
        } }
        #else
        META_Node(osgbDynamics,SoftBody)
        #endif
    virtual void traverse(osg::NodeVisitor&);


    ///set asscociated softbody
    ///if nodes are setted bake them to osg
    void setSoftBody(btSoftBody*body);
    inline const btSoftBody* getSoftBody()const{return _body;}
    inline btSoftBody* getSoftBody(){return _body;}

    const World *getParentWorld()const
    {
        return _parentWorld;
    }
    void setParentWorld(World *c)
    {
        _parentWorld=c;
    }

    ///mainly 4 missing bullet serialization
    void setWindVelocity(const osg::Vec3 &w);
    const osg::Vec3& getWindVelocity()const;
protected:

    virtual void addPhysicalObjectToParentWorld();
    ~SoftBody();
    btSoftBody *_body;
    World * _parentWorld;
    osg::Vec3 _windVelocity;///redondancy for serializer



};



// osgbDynamics
}


// __OSGBDYNAMICS_RIGID_BODY_H__
#endif
