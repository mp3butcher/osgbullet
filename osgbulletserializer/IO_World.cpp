/**
 * osgbDynamics serializer
 Julien Valentin
 */
#undef OBJECT_CAST
#define OBJECT_CAST dynamic_cast



#include "btWorldImporter.h"


#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>
#include <osgDB/FileUtils>



#include <osg/Camera>
#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

#include <sstream>

osgbDynamics::World*  sharedworld::currentworld=0;

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

sharedworld::currentworld= (&node);
   unsigned int sizel = 0; is >> sizel >> is.BEGIN_BRACKET;

    btBulletWorldImporter BulletImporter((	node.getDynamicsWorld()));

    char* memoryBuffer=new char[sizel];
    //is >>memoryBuffer;
    //is.readCharArray(memoryBuffer,sizel);
      char * ptr=memoryBuffer;

    while(ptr<memoryBuffer+sizel)
     is>>*ptr++;
	bool result = BulletImporter.loadFileFromMemory(memoryBuffer,sizel);


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
sharedworld::currentworld=const_cast<  osgbDynamics::World *>(&node);
    // start the serialization and serialize the trimeshShape
    serializer->startSerialization();
    const btDynamicsWorld * w=node.getDynamicsWorld();
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
    os <<serializer->getCurrentBufferSize() << os.BEGIN_BRACKET << std::endl;

  /*   char * portablestring=(char*)malloc(1+2*serializer->getCurrentBufferSize());
    char *outptr=portablestring;
    const unsigned char * ptr=serializer->getBufferPointer();

    while(ptr!=serializer->getBufferPointer()+ serializer->getCurrentBufferSize()){
   std::stringstream h; h<<std::hex<<short(*ptr++);
    *outptr++=h.str()[0];
    if(h.str().size()>1)*outptr++=h.str()[1];
    else *outptr++='0';
std::cout<<h.str()<<std::endl;;
    }
     *outptr='\0';
//CHAR_BIT

     //   os << serializer->getBufferPointer();
    outptr=portablestring;
    while(outptr!=portablestring+ 2*serializer->getCurrentBufferSize())
    os<<*outptr++;//.writeCharArray(portablestring,1+ serializer->getCurrentBufferSize()*2 );
*/
    const unsigned char * ptr=serializer->getBufferPointer();

    while(ptr<serializer->getBufferPointer()+ serializer->getCurrentBufferSize())
     os<<(char)*ptr++;
    os << os.END_BRACKET << std::endl;
// create a file and write the world to file
//FILE* file = fopen("testInit_Volume.bullet","wb");
//fwrite(serializer->getBufferPointer(),serializer->getCurrentBufferSize(),1, file);
//fclose(file);
return true;
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
    ADD_USER_SERIALIZER(PhysicalProps);
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
