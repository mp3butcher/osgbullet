/**
 * osgPhysics serializer
 Julien Valentin
*/

#include <osgbDynamics/SoftBody.h>

#include <osgbCollision/Utils.h>

#include <osgbDynamics/World.h>

#include "osgbtWorldImporter.h"

#include <btBulletFile.h>
#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>
#include <osgDB/FileUtils>


#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

namespace osgbDynamicsSoftBodyAnchorWrapper{

REGISTER_OBJECT_WRAPPER(osgbDynamicsSoftBodyAnchor,
                        new osgbDynamics::Anchor,
                        osgbDynamics::Anchor,
                        "osg::Object osgbDynamics::Anchor")
{

    ADD_INT_SERIALIZER( SoftBodyNodeIndex,0);
    ADD_VEC3F_SERIALIZER(LocalFrame,osg::Vec3());
    ADD_OBJECT_SERIALIZER(RigidBody,osgbDynamics::RigidBody,NULL);
    ADD_BOOL_SERIALIZER(IsCollisionEnable,true);

}

}


namespace osgbDynamicsSoftBodyWrapper{


static bool checkAnchors( const osgbDynamics::SoftBody& node )
{
    return node.getNumAnchors()>0;
}

static bool readAnchors( osgDB::InputStream& is, osgbDynamics::SoftBody& node )
{
    unsigned int size = 0; is >> size >> is.BEGIN_BRACKET;
    for ( unsigned int i=0; i<size; ++i )
    {
        osg::ref_ptr<osg::Object> obj = is.readObject();
        osgbDynamics::Anchor* child = dynamic_cast<osgbDynamics::Anchor*>( obj.get() );
        if ( child ) node.addAnchor( child );
if(child->getRigidBody()->getRigidBody() &&node.getSoftBody())
        node.getSoftBody()->appendAnchor( child->getSoftBodyNodeIndex(),child->getRigidBody()->getRigidBody(),osgbCollision::asBtVector3(child->getLocalFrame())
    ,!child->getIsCollisionEnable() );   ///bool disableCollisionBetweenLinkedBodies=false,btScalar influence = 1);
    else{
    OSG_WARN<<" Warning: Anchor can't be defined (missing bodies definition)"<<std::endl;
    }
    }
    is >> is.END_BRACKET;
    return true;
}

static bool writeAnchors( osgDB::OutputStream& os, const osgbDynamics::SoftBody& node )
{
    unsigned int size = node.getNumAnchors();
    os << size << os.BEGIN_BRACKET << std::endl;
    for ( unsigned int i=0; i<size; ++i )
    {
        os << node.getAnchor(i);
    }
    os << os.END_BRACKET << std::endl;
    return true;
}

static bool checkPhysicalProps( const osgbDynamics::SoftBody& node )
{
    return true;
}

static bool readPhysicalProps( osgDB::InputStream& is, osgbDynamics::SoftBody& node )
{

   unsigned int sizel = 0; is >> sizel >> is.BEGIN_BRACKET;
   const btSoftRigidDynamicsWorld* s=dynamic_cast<const btSoftRigidDynamicsWorld*>(sharedworld::currentworld->getDynamicsWorld());
   btSoftRigidDynamicsWorld* w=0;
   if(s)w=const_cast<btSoftRigidDynamicsWorld*>(s);
unsigned int cpt=w->getSoftBodyArray().size();
    osgbtBulletWorldImporter BulletImporter(w);

    char* memoryBuffer=new char[sizel];
    //is >>memoryBuffer;
 char* ptr=memoryBuffer;
    while(ptr!=memoryBuffer+sizel)
     is>>*ptr++;
	//bool result = BulletImporter.loadFileFromMemory(memoryBuffer,sizel);
bParse::btBulletFile* bulletFile2 = new bParse::btBulletFile(memoryBuffer,sizel);

	 bool result = BulletImporter.loadFileFromMemory(bulletFile2);
/*bool ok = (bulletFile2->getFlags()& bParse::FD_OK)!=0;

	if (ok)
		bulletFile2->parse(m_verboseMode);
	else
		return false;

	if (m_verboseMode & bParse::FD_VERBOSE_DUMP_CHUNKS)
	{
		bulletFile2->dumpChunks(bulletFile2->getFileDNA());
	}
	result=BulletImporter.convertAllObjects(bulletFile2);
	delete bulletFile2;
*/


    is >> is.END_BRACKET;

    if( cpt+1==w->getSoftBodyArray().size()){
btSoftBody*read= static_cast<btSoftBody*>(w->getSoftBodyArray()[cpt]);
    node.setSoftBody(read);
    w->removeSoftBody(read);
    return true;
    }else return false;
}


static bool writePhysicalProps( osgDB::OutputStream& os, const osgbDynamics::SoftBody& node ){
    btDefaultSerializer* serializer = new btDefaultSerializer();
    // start the serialization and serialize the trimeshShape
    ///Bullet serialization is shitty but don't have time to do that in a proper way (bt->osg wrapping)
    serializer->startSerialization();
    const btDiscreteDynamicsWorld * w=sharedworld::currentworld->getDynamicsWorld();

    const btSoftBody *rig=node.getSoftBody();
    btCollisionObject *colObj=const_cast<btSoftBody*>(node.getSoftBody());
			int len = colObj->calculateSerializeBufferSize();
			btChunk* chunk = serializer->allocate(len,1);
			const char* structType = colObj->serialize(chunk->m_oldPtr, serializer);
			serializer->finalizeChunk(chunk,structType,BT_SOFTBODY_CODE,colObj);


    serializer->finishSerialization();

    os << serializer->getCurrentBufferSize() << os.BEGIN_BRACKET << std::endl;

     //   os << serializer->getBufferPointer();
   // os.writeCharArray((char*)serializer->getBufferPointer(), serializer->getCurrentBufferSize() );
 const unsigned char * ptr=serializer->getBufferPointer();
    while(ptr!=serializer->getBufferPointer()+ serializer->getCurrentBufferSize())
     os<<*ptr++;

    os << os.END_BRACKET << std::endl;
// create a file and write the world to file
//FILE* file = fopen("testInit_Volume.bullet","wb");
//fwrite(serializer->getBufferPointer(),serializer->getCurrentBufferSize(),1, file);
//fclose(file);
return true;
}


REGISTER_OBJECT_WRAPPER(osgbDynamicsSoftBody,
                        new osgbDynamics::SoftBody,
                        osgbDynamics::SoftBody,
                        "osg::Object osg::Node osg::Drawable osg::Geometry osgbDynamics::SoftBody")
{

    ADD_USER_SERIALIZER( PhysicalProps);
    ADD_VEC3F_SERIALIZER(WindVelocity,osg::Vec3());
    ADD_USER_SERIALIZER(Anchors);

}
}
