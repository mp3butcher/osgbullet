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
// Written by J Valentin

#ifndef OSGBDYNAMICS_BTWORLD
#define OSGBDYNAMICS_BTWORLD 1

#include <osg/Node>
#include <osg/ShapeDrawable>

#include <osgbDynamics/RigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

namespace osgbDynamics
{



/** The basic element of the physics abstract layer */
class OSGBDYNAMICS_EXPORT  World : public osg::NodeCallback
{
public:
    typedef enum
    {
        RIGID_ONLY,
        RIGID_AND_SOFT
    } WorldType;
    World();
    World( const World& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
META_Object(osgbDynamics,World)
   /* virtual bool isSameKindAs( const osg::Object* obj ) const
    {
        return dynamic_cast<const World*>(obj)!=NULL;
    }
    virtual const char* libraryName () const
    {
        return "osgbDynamics";
    }
    virtual const char* className () const
    {
        return "World";
    }*/

    btDynamicsWorld * getDynamicsWorld()
    {
        return _btworld;
    }
    const btDynamicsWorld * getDynamicsWorld()const
    {
        return _btworld;
    }
    void  setDynamicsWorld(btDynamicsWorld *w)
    {
        _btworld=w  ;
    }

    WorldType getWorldType()const
    {
        return _worldtype;
    }
    void setWorldType(WorldType t);

    /** Update the element */
    virtual void update( osg::Node* node, osg::NodeVisitor* nv );

    /** Do some post work if required */
    //virtual void postevent( osg::Node* node, osg::NodeVisitor* nv ) {}

    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );


    unsigned int getNumPhysicalObjects()const
    {
        return _managed.size();
    }
    const PhysicalObject * getPhysicalObject(unsigned int i)const
    {
        return _managed[i];
    }
    PhysicalObject * getPhysicalObject(unsigned int i)
    {
        return _managed[i];
    }
    void addPhysicalObject(PhysicalObject*p)
    {
        _managed.push_back(p);
    }
    void removePhysicalObject(PhysicalObject*p)
    {
        for(std::vector<osg::ref_ptr<PhysicalObject> >::iterator i=_managed.begin(); i!=_managed.end(); i++)
        {
            if(i->get()==p)
            {
                _managed.erase(i);
                return;
            }
        }
    }

    double getDeltaTime()const
    {
        return _deltaTime;
    }
    void setDeltaTime(double d)
    {
        _deltaTime=d;
    }


protected:
    virtual ~ World();
    WorldType _worldtype;
    double _deltaTime,_prevousReferenceTime;
    btDynamicsWorld* _btworld;
    std::vector<osg::ref_ptr<PhysicalObject> > _managed;
};

}

#endif
