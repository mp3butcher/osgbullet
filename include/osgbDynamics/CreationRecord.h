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

#ifndef __OSGBDYNAMICS_CREATION_RECORD_H__
#define __OSGBDYNAMICS_CREATION_RECORD_H__ 1


#include <osgbDynamics/Export.h>
#include <osgbCollision/CollisionShapes.h>

#include <osg/Object>
#include <osg/Matrix>
#include <osg/Vec3>

#include <btBulletDynamicsCommon.h>

#include <string>



namespace osgbDynamics
{



/** \class CreationRecord CreationRecord.h <osgbDynamics/CreationRecord.h>
\brief Data record for rigid body construction information.

Fill in this struct and pass it to the createRigidBody() functions
to specify rigid body (and collision shape) construction parameters.
See the \link rigidbody rigid body creation utilities \endlink
for more information.

This record can be stored in UserData on the rigid body subgraph
root node to facilitate saving and restoring physics state.

This struct can be serialized to/from the .osg file format. See the
osgdb_osgbDynamics plugin. */
struct OSGBDYNAMICS_EXPORT CreationRecord : public osg::Object
{
    CreationRecord();
    CreationRecord( const CreationRecord& rhs, osg::CopyOp copyop=osg::CopyOp::SHALLOW_COPY );

    META_Object(osgbDynamics,CreationRecord);

    void setSceneGraphNode(osg::Node* s ){_sceneGraph=s;}
    osg::Node* getSceneGraphNode()const{return _sceneGraph;}
    osg::Node* _sceneGraph;

    /** The dot OSG representation of this class maintains a version number for
    debugging purposes, allow deprecation or old data, and allow introduction
    of new data. Do not explicitly set this value. The constructor initializes
    this to the current version, and the dot OSG read function Creation_readLocalData()
    will set it based on the value found in the dot OSG file being loaded. */
    unsigned int _version;

    /** Specify the center of mass. If not set, osgbDynamics::createRigidBody
    will use the scene graph bounding volume center as the center of mass. */
    void setCenterOfMass( const osg::Vec3& com );
    const osg::Vec3& getCenterOfMass()const{return _com;}
    osg::Vec3 _com;
    void setIsCOMSet( bool com ){_comSet=com;}
    bool getIsCOMSet()const{return _comSet;}
    bool _comSet;

    void setMass( float com ){_mass=com;}
    float getMass()const{return _mass;}
    float _mass;



    /** Specify the collision shape margin for convex hull and convex tri mesh
    collision shapes.

    Note: Margin is currently used only if _overall is true. */
    void setMargin(  float margin );
    float getMargin()const{return _margin;}
    float _margin;



    void setScale( const osg::Vec3& com ){_scale=com;};
    const osg::Vec3& getScale()const{return _scale;}
    osg::Vec3 _scale;

    void setParentTransform( const osg::Matrix& com ){_parentTransform=com;};
    const osg::Matrix& getParentTransform()const{return _parentTransform;}
    osg::Matrix _parentTransform;


    bool _marginSet;
    BroadphaseNativeTypes _shapeType;

    void setRestitution(  float f ){_restitution=f;};
    float getRestitution()const{return _restitution;}
    float _restitution;

    void setFriction(  float f ){_friction=f;};
    float getFriction()const{return _friction;}
    float _friction;


    void setRollingFriction(  float f ){_rollingFriction=f;};
    float getRollingFriction()const{return _rollingFriction;}
    float _rollingFriction;


    void setLinearDamping(  float f ){_linearDamping=f;};
    float getLinearDamping()const{return _linearDamping;}
    float _linearDamping;
    void setAngularDamping(  float f ){_angularDamping=f;};
    float getAngularDamping()const{return _angularDamping;}
    float _angularDamping;

    /** For _shapeType == CYLINDER_SHAPE_PROXYTYPE only. */
    osgbCollision::AXIS _axis;

    bool _overall;
    /** Corresponds to the \c _reductionLevel parameter for
    osgbCollision::btCompoundShapeFromOSGGeodes(). */
    typedef enum {
        NONE = 0,
        MINIMAL = 1,
        INTERMEDIATE = 2,
        AGGRESSIVE = 3
    } ReductionLevel;
    /** \brief Specify optional geometry reduction. Default is NONE.
    */
    void setReductionLevel(ReductionLevel com ){_reductionLevel=com;};
    ReductionLevel getReductionLevel()const{return _reductionLevel;}
    ReductionLevel _reductionLevel;

};


// osgbDynamics
}


// __OSGBDYNAMICS_CREATION_RECORD_H__
#endif
