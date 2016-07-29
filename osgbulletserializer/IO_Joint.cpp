/**
 * osgbDynamics serializer
 Julien Valentin
*/

#include <osgbDynamics/Joint.h>

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

static bool checkPhysicalProps( const osgbDynamics::Joint& node )
{
    return true;
}

static bool readPhysicalProps( osgDB::InputStream& is, osgbDynamics::Joint& node )
{

    unsigned int sizel = 0;
    is >> sizel >> is.BEGIN_BRACKET;
    btDiscreteDynamicsWorld *w=const_cast<btDiscreteDynamicsWorld*>(sharedworld::currentworld->getDynamicsWorld());
    //btDiscreteDynamicsWorld *w=new btDiscreteDynamicsWorld();

    unsigned int cpt=w->getNumConstraints();
    osgbtBulletWorldImporter BulletImporter(w);

    char* memoryBuffer=new char[sizel];
    //is >>memoryBuffer;
    char* ptr=memoryBuffer;
    while(ptr!=memoryBuffer+sizel)
        is>>*ptr++;
    //bool result = BulletImporter.loadFileFromMemory(memoryBuffer,sizel);
    bParse::btBulletFile* bulletFile2 = new bParse::btBulletFile(memoryBuffer,sizel);

 //   bool result = BulletImporter.loadFileFromMemory(bulletFile2);
    bool ok = (bulletFile2->getFlags()& bParse::FD_OK)!=0;

    if (ok)
        bulletFile2->parse(false);
    else
        return false;

    //if (m_verboseMode & bParse::FD_VERBOSE_DUMP_CHUNKS)
    {
        bulletFile2->dumpChunks(bulletFile2->getFileDNA());
    }
    //result=BulletImporter.convertAllObjects(bulletFile2);

    for (int i=0; i<bulletFile2->m_constraints.size(); i++)
    {
        btTypedConstraintData2* constraintData = (btTypedConstraintData2*)bulletFile2->m_constraints[i];
        btTypedConstraintFloatData* singleC = (btTypedConstraintFloatData*)bulletFile2->m_constraints[i];
        btTypedConstraintDoubleData* doubleC = (btTypedConstraintDoubleData*)bulletFile2->m_constraints[i];

        //btCollisionObject** colAptr = m_bodyMap.find(constraintData->m_rbA);
        //btCollisionObject** colBptr = m_bodyMap.find(constraintData->m_rbB);

        btRigidBody* rbA = node.getBodyA()->getRigidBody();
        btRigidBody* rbB =0;
        if( node.getBodyB())
        rbB=node.getBodyB()->getRigidBody();


        if (!rbA && !rbB)
            continue;

        bool isDoublePrecisionData = (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)!=0;

        if (isDoublePrecisionData)
        {
            if (bulletFile2->getVersion()>=282)
            {
                btTypedConstraintDoubleData* dc = (btTypedConstraintDoubleData*)constraintData;
                BulletImporter.convertConstraintDouble(dc, rbA,rbB, bulletFile2->getVersion());
            } /*else
			{
				//double-precision constraints were messed up until 2.82, try to recover data...

				btTypedConstraintData* oldData = (btTypedConstraintData*)constraintData;

				BulletImporter.convertConstraintBackwardsCompatible281(oldData, rbA,rbB, bulletFile2->getVersion());

			}*/
        }
        else
        {
            btTypedConstraintFloatData* dc = (btTypedConstraintFloatData*)constraintData;
            BulletImporter.convertConstraintFloat(dc, rbA,rbB, bulletFile2->getVersion());
        }


    }
    delete bulletFile2;



    is >> is.END_BRACKET;

    if( cpt+1==w->getNumConstraints())
    {
btTypedConstraint* read=const_cast<btTypedConstraint*>(w->getConstraint(cpt));
        node.setConstraint( read);
        w->removeConstraint(read);

    std::cout<<cpt<<w->getNumCollisionObjects()<<std::endl;
        return true;
    }
    else return false;
}


static bool writePhysicalProps( osgDB::OutputStream& os, const osgbDynamics::Joint& node )
{
    btDefaultSerializer* serializer = new btDefaultSerializer();
    // start the serialization and serialize the trimeshShape
    ///Bullet serialization is shitty but don't have time to do that in a proper way (bt->osg wrapping)
    serializer->startSerialization();
    const btDiscreteDynamicsWorld * w=sharedworld::currentworld->getDynamicsWorld();

    const btTypedConstraint *rig=node.getConstraint();
    btTypedConstraint *colObj=const_cast<btTypedConstraint*>(rig);
    int len = colObj->calculateSerializeBufferSize();
    btChunk* chunk = serializer->allocate(len,1);
    const char* structType = colObj->serialize(chunk->m_oldPtr, serializer);
    serializer->finalizeChunk(chunk,structType,BT_CONSTRAINT_CODE,colObj);


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


REGISTER_OBJECT_WRAPPER(osgbDynamicsJoint,
                        new osgbDynamics::Joint,
                        osgbDynamics::Joint,
                        "osg::Object osgbDynamics::Joint")
{
    ADD_OBJECT_SERIALIZER(BodyA,osgbDynamics::RigidBody,NULL);
    ADD_OBJECT_SERIALIZER(BodyB,osgbDynamics::RigidBody,NULL);
    ADD_USER_SERIALIZER( PhysicalProps);


}
