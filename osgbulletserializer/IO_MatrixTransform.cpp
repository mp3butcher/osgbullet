/**
 * osgPhysics serializer
 Julien Valentin

*/

#include <osgbDynamics/RigidBodyAnimation.h>

#undef OBJECT_CAST
#define OBJECT_CAST dynamic_cast
#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>
#include <osgDB/FileUtils>


#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>


/*

struct MatrixTransformFinishedObjectReadCallback : public osgDB::FinishedObjectReadCallback
{
    virtual void objectRead(osgDB::InputStream&, osg::Object& obj)
    {

        osgPhysics::MatrixTransform& trans = static_cast<osgPhysics::MatrixTransform&>(obj);
        ///get the matrix from Attribute map else set as identity
        osgPhysics::BaseElement* element= dynamic_cast<osgPhysics::BaseElement*>(trans.getUpdateCallback());
        if(element && element->getAttributeMap())
        {
            osg::Matrixd m;
            element->getAttributeMap()->getAttribute("matrix",m);
            trans.setMatrix(m);
        }
    }
};
*/
REGISTER_OBJECT_WRAPPER(osgbDynamicsRigidBodyAnimation,
                        new osgbDynamics::RigidBodyAnimation,
                        osgbDynamics::RigidBodyAnimation,
                        "osg::Object osg::Callback osg::NodeCallback osgbDynamics::RigidBodyAnimation")
{


}
#undef OBJECT_CAST
#define OBJECT_CAST static_cast

