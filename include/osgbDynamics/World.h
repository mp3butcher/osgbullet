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

#include <osg/Group>
#include <osg/ShapeDrawable>

#include <osgbDynamics/RigidBody.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
namespace osgbCollision{
class GLDebugDrawer;
}
namespace osgbDynamics
{




/** The basic element of the physics abstract layer */
class OSGBDYNAMICS_EXPORT  World : public osg::Group
{
public:
    typedef enum
    {
        RIGID_ONLY,
        RIGID_AND_SOFT
    } WorldType;
    World();
    World( const World& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
  virtual osg::Object* cloneType() const { return new World (); }
        virtual osg::Object* clone(const osg::CopyOp& copyop) const { return new World (*this,copyop); }
        virtual bool isSameKindAs(const osg::Object* obj) const { return dynamic_cast<const World *>(obj)!=NULL; }
        virtual const char* className() const { return "World"; }
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



    btDiscreteDynamicsWorld * getDynamicsWorld()
    {
        return _btworld;
    }
    const btDiscreteDynamicsWorld * getDynamicsWorld()const
    {
        return _btworld;
    }
    void  setDynamicsWorld(btDiscreteDynamicsWorld *w)
    {
        _btworld=w  ;
    }

    WorldType getWorldType()const
    {
        return _worldtype;
    }
    void setWorldType(WorldType t);

    /** Update the element */
//   virtual void update( osg::Node* node, osg::NodeVisitor* nv );

    /** Do some post work if required */
    //virtual void postevent( osg::Node* node, osg::NodeVisitor* nv ) {}

//    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );


     unsigned int getNumJoints()const
    {
        return _joints.size();
    }
   const Joint * getJoint(unsigned int i)const
    {
        return _joints[i];
    }
    Joint * getJoint(unsigned int i)
    {
        return _joints[i];
    }
    void addJoint(Joint*p);
    void removeJoint(Joint*p);

    double getDeltaTime()const
    {
        return _deltaTime;
    }
    void setDeltaTime(double d)
    {
        _deltaTime=d;
    }

    void setDebugEnabled(bool b);

    bool getDebugEnabled()const{return _debugdraw;}


    virtual void traverse(osg::NodeVisitor &nv);


protected:
    virtual ~ World();
    bool _debugdraw;
    WorldType _worldtype;
    double _deltaTime,_prevousReferenceTime;
    btDiscreteDynamicsWorld* _btworld;
    std::vector<osg::ref_ptr<Joint> > _joints;
    std::vector<osg::ref_ptr<Joint> > _joints2add; ///temporary store in case world isn't setted  (parsed on update traversal)
    ///inner debug draw
    osg::ref_ptr<osg::Group> _debugdrawable;
    osgbCollision::GLDebugDrawer* dbgDraw;
};
///Node visitor used to find world..default is TRAVERSE_PARENTS
class OSGBDYNAMICS_EXPORT WorldFinderVisitor : public osg::NodeVisitor
{
public:
    WorldFinderVisitor()
        : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_PARENTS), foundWorld(NULL)  {}
    void apply( osg::Node& node )
    {
        if( !foundWorld )
            foundWorld = dynamic_cast<World*>(&node);
        traverse( node );
    }
    World* getFoundWorld()const{return  foundWorld;}
    protected:
        World* foundWorld;
};
}

#endif
