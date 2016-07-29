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

#include <osgbDynamics/SoftBody.h>
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
using namespace osgbCollision;
namespace osgbDynamics
{

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


SoftBody::SoftBody():_parentWorld(0),_body(0) {
    //forceUpdate
    setNumChildrenRequiringUpdateTraversal(1);
}
SoftBody::~SoftBody() {}
SoftBody::SoftBody( const SoftBody& copy, const osg::CopyOp& copyop) {}
/*const btSoftBody* SoftBody::getSoftBody()const
{
    const   osgbCollision::RefBulletObject<  btSoftBody >*  refptr=
        dynamic_cast<  const   osgbCollision::RefBulletObject<  btSoftBody >* >( getUserData() );

    return (refptr?refptr->get():0);
}

btSoftBody* SoftBody::getSoftBody()
{
    osgbCollision::RefBulletObject<  btSoftBody >*  refptr=
        dynamic_cast<     osgbCollision::RefBulletObject<  btSoftBody >* >( getUserData() );

    return (refptr?refptr->get():0);
}
*/
void SoftBody::setSoftBody( btSoftBody *softBody )
{
    _body=softBody;//setUserData(new osgbCollision::RefBulletObject<btSoftBody>(softBody));
    _windVelocity=asOsgVec3((softBody)->getWindVelocity());

   if(softBody->m_tetras.size()>0)
    {
        ///tetramesh detected: TODO

    }
    else    if(softBody->m_faces.size()>0)
    {
        ///trimesh detected so bake nodes to geometry
#define FILLINDEX(TYPE)  { osg::DrawElements##TYPE * primset=new osg::DrawElements##TYPE(GL_TRIANGLES);\
    for(int i=0;i<softBody->m_faces.size();i++)\
        for(short ni=0;ni<3;ni++)\
            primset->push_back(softBody->m_faces[i].m_n[ni]-&softBody->m_nodes[0]);\
    addPrimitiveSet(primset);}
        this->removePrimitiveSet(0,getNumPrimitiveSets());
        unsigned int numIndices=softBody->m_faces.size()*3;
        if(numIndices<128)          FILLINDEX(UShort)
            else if(numIndices<65535)   FILLINDEX(UByte)
                else                        FILLINDEX(UInt)
#undef FILLINDEX
        osg::Vec3Array *verts=new osg::Vec3Array();
        osg::Vec3Array *norms=new osg::Vec3Array();
        verts->resize(softBody->m_nodes.size());
        norms->resize(softBody->m_nodes.size());
        setVertexArray(verts);
        setNormalArray(norms);
    }
    else if(softBody->m_links.size()>0)
    {
        ///rope detected
#define FILLINDEX(TYPE)  { osg::DrawElements##TYPE * primset=new osg::DrawElements##TYPE(GL_LINES);\
    for(int i=0;i<softBody->m_links.size();i++)\
        for(short ni=0;ni<2;ni++)\
            primset->push_back(softBody->m_links[i].m_n[ni]-&softBody->m_nodes[0]);\
    addPrimitiveSet(primset);}
        this->removePrimitiveSet(0,getNumPrimitiveSets());
        unsigned int numIndices=softBody->m_links.size()*2;
        if(numIndices<128)          FILLINDEX(UShort)
            else if(numIndices<65535)   FILLINDEX(UByte)
                else                        FILLINDEX(UInt)
        osg::Vec3Array *verts=new osg::Vec3Array();
        osg::Vec3Array *norms=new osg::Vec3Array();
        verts->resize(softBody->m_nodes.size());
        norms->resize(softBody->m_nodes.size());
        setVertexArray(verts);
        setNormalArray(norms);
#undef FILLINDEX
    }
getOrCreateVertexBufferObject()->setUsage( GL_DYNAMIC_DRAW );


}
void SoftBody::setWindVelocity(const osg::Vec3& w){
    btSoftBody*_body=getSoftBody();
    if(_body){
        _windVelocity=w;
        _body->setWindVelocity(asBtVector3(w));
        }
}
const osg::Vec3& SoftBody::getWindVelocity()const {
    const btSoftBody*_body=getSoftBody();
    if(_body)
        assert(_windVelocity==asOsgVec3(const_cast<btSoftBody*>(_body)->getWindVelocity()));
    return _windVelocity;
}
void SoftBody::traverse(osg::NodeVisitor &nv)
{
   switch(nv.getVisitorType())
    {
    case osg::NodeVisitor::UPDATE_VISITOR:
    //case osg::NodeVisitor::NODE_VISITOR:
    {
 if ( !_parentWorld)
        {
            if(getNumParents()!=0)
            {
                FindParentVisitor fpv;
                this->accept(fpv);
                if(fpv.foundWorld){
                _parentWorld=fpv.foundWorld;
                addPhysicalObjectToParentWorld();
                }else{
                 OSG_WARN<<"SoftBody: parent world not foudn"<<std::endl;
                }
            }
        }

        ///updateVertices according btDoftBody Nodes
        osg::Vec3Array* verts( dynamic_cast< osg::Vec3Array* >( getVertexArray() ) );

        // Update the vertex array from the soft body node array.
        const btSoftBody * _softBody=getSoftBody();
        const btSoftBody::tNodeArray& nodes = _softBody->m_nodes;
        //if(verts->size()<nodes.size())verts->resize(nodes.size());
        osg::Vec3Array::iterator it( verts->begin() );
        unsigned int idx;
        for( idx=0; idx<nodes.size(); idx++)
        {
            *it++ =  osgbCollision::asOsgVec3( nodes[ idx ].m_x );
//OSG_WARN<<*(it-1)<<std::endl;
        }
        verts->dirty();
        dirtyBound();
//OSG_WARN<<"SoftBody: dirty"<<std::endl;

        // Generate new normals.
        osgUtil::SmoothingVisitor smooth;
        smooth.smooth( *this );
        getNormalArray()->dirty();
    }
    }
    osg::Geometry::traverse(   nv );
}
void SoftBody::addPhysicalObjectToParentWorld()
{
    if(_parentWorld)
    {
        btSoftBody*_body=getSoftBody();
        if(_body)
        {
            btSoftRigidDynamicsWorld* w=dynamic_cast<   btSoftRigidDynamicsWorld*>(_parentWorld->getDynamicsWorld());
            if(w)            if(w->getSoftBodyArray().findLinearSearch(_body) == w->getSoftBodyArray().size())
                w->addSoftBody(_body);
            else
            {
                OSG_WARN<<"SoftBody: try to add SoftBody to a rigid word"<<std::endl;
            }

        }
        else
        {
            OSG_WARN<<"SoftBody: btSoftBody is not setted"<<std::endl;
        }
    }
    else
    {
        OSG_WARN<<"SoftBody: parentworld hasn't been found"<<std::endl;
    }

}

}


