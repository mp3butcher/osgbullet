/**
 * osgbDynamics serializer
 Julien Valentin
 */
#include "btWorldImporter.h"

#include "BulletSoftBody/btSoftBodyData.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

class btBulletFile;




namespace bParse
{
	class btBulletFile;

};



///The btBulletWorldImporter is a starting point to import .bullet files.
///note that not all data is converted yet. You are expected to override or modify this class.

class btBulletWorldImporter : public btWorldImporter
{
protected:
	btSoftRigidDynamicsWorld* m_softRigidWorld;

	btHashMap<btHashPtr,btSoftBody::Material*>	m_materialMap;

	btHashMap<btHashPtr,btSoftBody*>	m_clusterBodyMap;
	btHashMap<btHashPtr,btSoftBody*>	m_softBodyMap;

public:

	btBulletWorldImporter(btDynamicsWorld* world=0);

	virtual ~btBulletWorldImporter();

	///if you pass a valid preSwapFilenameOut, it will save a new file with a different endianness
	///this pre-swapped file can be loaded without swapping on a target platform of different endianness
	bool	loadFile(const char* fileName, const char* preSwapFilenameOut=0);

	///the memoryBuffer might be modified (for example if endian swaps are necessary)
	bool	loadFileFromMemory(char *memoryBuffer, int len);

	bool	loadFileFromMemory(bParse::btBulletFile* file);

	//call make sure bulletFile2 has been parsed, either using btBulletFile::parse or btBulletWorldImporter::loadFileFromMemory
	virtual	bool	convertAllObjects(bParse::btBulletFile* file);




};
#undef OBJECT_CAST
#define OBJECT_CAST dynamic_cast

#include <osgbDynamics/World.h>


#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>
#include <osgDB/FileUtils>



#include <osg/Camera>
#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>




using namespace osgbDynamics;
using namespace osg;
using namespace osgDB;

namespace  osgbDynamicsWorldwrapper
{
/*
PHYPROPERTY(osgPhysics::World,Gravity,osg::Vec3)*/
static bool checkPhysicalObjects( const osgbDynamics::World& node )
{
    return node.getNumPhysicalObjects()>0;
}

static bool readPhysicalObjects( osgDB::InputStream& is, osgbDynamics::World& node )
{
    unsigned int size = 0; is >> size >> is.BEGIN_BRACKET;
    for ( unsigned int i=0; i<size; ++i )
    {
        osg::ref_ptr<osg::Object> obj = is.readObject();
        osgbDynamics::PhysicalObject* child = dynamic_cast<osgbDynamics::PhysicalObject*>( obj.get() );
        if ( child ) node.addPhysicalObject( child );
    }
    is >> is.END_BRACKET;
    return true;
}

static bool writePhysicalObjects( osgDB::OutputStream& os, const osgbDynamics::World& node )
{
    unsigned int size = node.getNumPhysicalObjects();
    os << size << os.BEGIN_BRACKET << std::endl;
    for ( unsigned int i=0; i<size; ++i )
    {
        os << node.getPhysicalObject(i);
    }
    os << os.END_BRACKET << std::endl;
    return true;
}

static bool checkPhysicalProps( const osgbDynamics::World& node )
{
    return true;
}

static bool readPhysicalProps( osgDB::InputStream& is, osgbDynamics::World& node )
{
    unsigned int size = 0; is >> size >> is.BEGIN_BRACKET;

    btBulletWorldImporter BulletImporter((	node.getDynamicWorld()));

    char* memoryBuffer=new char[size];
    //is >>memoryBuffer;
    is.readCharArray(memoryBuffer,size);
	bool result = BulletImporter.loadFileFromMemory(memoryBuffer,size);


    is >> is.END_BRACKET;
    return true;
}

class btDiscreteDynamicsWorldHacker :public btDiscreteDynamicsWorld{
public:

	void	serializeWorldInfo(btSerializer* serializer){
	serializeDynamicsWorldInfo( serializer);}

};

static bool writePhysicalProps( osgDB::OutputStream& os, const osgbDynamics::World& node ){
    btDefaultSerializer* serializer = new btDefaultSerializer();

    // start the serialization and serialize the trimeshShape
    serializer->startSerialization();
    const btDynamicsWorld * w=node.getDynamicWorld();
    static_cast<btDiscreteDynamicsWorldHacker*>(const_cast<btDynamicsWorld *>(w))->serializeWorldInfo(serializer);


    /*btCollisionObject *colObj=psb;
                int len = colObj->calculateSerializeBufferSize();
                btChunk* chunk = serializer->allocate(len,1);
                const char* structType = colObj->serialize(chunk->m_oldPtr, serializer);
                serializer->finalizeChunk(chunk,structType,BT_SOFTBODY_CODE,colObj);
    */


    //psb->serializeSingleObject(serializer);
    //psb->getCollisionShape()->serializeSingleShape(serializer);

    serializer->finishSerialization();

    os << serializer->getCurrentBufferSize() << os.BEGIN_BRACKET << std::endl;

     //   os << serializer->getBufferPointer();
    os.writeCharArray((char*)serializer->getBufferPointer(), serializer->getCurrentBufferSize() );

    os << os.END_BRACKET << std::endl;
// create a file and write the world to file
//FILE* file = fopen("testInit_Volume.bullet","wb");
//fwrite(serializer->getBufferPointer(),serializer->getCurrentBufferSize(),1, file);
//fclose(file);

}

REGISTER_OBJECT_WRAPPER(osgbDynamicsWorld,
                        new osgbDynamics::World,
                        osgbDynamics::World,
                        "osg::Object osg::Callback osg::NodeCallback osgbDynamics::World")
{

   // ADD_STRING_SERIALIZER(PhysicalEngineString,"");
   // ADD_USER_SERIALIZER(Gravity);
   BEGIN_ENUM_SERIALIZER(WorldType,RIGID_ONLY);
   ADD_ENUM_VALUE(RIGID_ONLY);
   ADD_ENUM_VALUE(RIGID_AND_SOFT);
   END_ENUM_SERIALIZER();
    ADD_USER_SERIALIZER(PhysicalObjects);



}
}
/*
namespace osgphysicsCollisionMaskwrapper
{
REGISTER_OBJECT_WRAPPER(osgPhysicscollmask ,
                       new            osgPhysics::CollisionMask,
                        osgPhysics::CollisionMask,
                        "osg::Object  osgPhysics::CollisionMask"){

                        ADD_UINT_SERIALIZER(GroupID,0);
                        ADD_UINT_SERIALIZER(Mask,0);
                        }
}
namespace osgphysicsBaseElementwrapper
{
REGISTER_OBJECT_WRAPPER(osgPhysicsJBaseElement,
                       NULL,
                        osgPhysics::BaseElement,
                        "osg::Object  osg::NodeCallback  osgPhysics::BaseElement"){

                        ADD_OBJECT_SERIALIZER(CollisionMask, osgPhysics::CollisionMask,NULL);
                        }
}*/
//MY
#undef OBJECT_CAST
#define OBJECT_CAST static_cast
