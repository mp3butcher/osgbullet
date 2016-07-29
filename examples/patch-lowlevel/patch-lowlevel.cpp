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

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osg/Geode>
#include <osg/LightModel>
#include <osg/Texture2D>
#include <osgUtil/SmoothingVisitor>

#include <osgbDynamics/GroundPlane.h>
#include <osgbDynamics/SoftBody.h>
#include <osgbDynamics/World.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Utils.h>
#include <osgbInteraction/DragHandler.h>
#include <osgbInteraction/LaunchHandler.h>
//#include <osgbInteraction/SaveRestoreHandler.h>

//#include <osgwTools/Shapes.h>
#include <osg/ShapeDrawable>

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftBody.h>

#include <osg/io_utils>
#include <string>


#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
btSoftBodyWorldInfo	worldInfo;


/** \cond */
struct MeshUpdater : public osg::Drawable::UpdateCallback
{
    MeshUpdater( const btSoftBody* softBody )
      : _softBody( softBody )
    {}
    virtual ~MeshUpdater()
    {}

    virtual void update( osg::NodeVisitor*, osg::Drawable* draw )
    {
        osg::Geometry* geom( draw->asGeometry() );
        osg::Vec3Array* verts( dynamic_cast< osg::Vec3Array* >( geom->getVertexArray() ) );

        // Update the vertex array from the soft body node array.
        osg::Matrix m=osgbCollision::asOsgMatrix(_softBody->getWorldTransform());
        const btSoftBody::tNodeArray& nodes = _softBody->m_nodes;
        //if(verts->size()<nodes.size())verts->resize(nodes.size());
        osg::Vec3Array::iterator it( verts->begin() );
        unsigned int idx;
        for( idx=0; idx<nodes.size(); idx++)
        {
            *it++ =m* osgbCollision::asOsgVec3( nodes[ idx ].m_x );
        }
        verts->dirty();
        draw->dirtyBound();

        // Generate new normals.
        osgUtil::SmoothingVisitor smooth;
        smooth.smooth( *geom );
        geom->getNormalArray()->dirty();
    }

    const btSoftBody* _softBody;

};
/** \endcond */

    osg::Node* makeFlag( btSoftRigidDynamicsWorld* bw )
{
    const short resX( 12 ), resY( 9 );

    osg::ref_ptr< osg::Geode > geode( new osg::Geode );

      osg::Vec3 llCorner( -2., 0., 5. );
      osg::Vec3 uVec( 4., 0., 0. );
      osg::Vec3 vVec( 0., 0.1, 3. ); // Must be at a slight angle for wind to catch it.
    osgbDynamics::SoftBody* geom = new osgbDynamics::SoftBody;
    //osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array * verts=new osg::Vec3Array();
    osg::Vec3Array * norms=new osg::Vec3Array();
    osg::Vec2Array * texarray=new osg::Vec2Array();
    for(short j=0;j<resY;j++)
    {
        const float	ty=j/(float)(resY-1);
        for(short i=0;i<resX;i++)
        {
            const float	tx=i/(float)(resX-1);
            texarray->push_back(osg::Vec2(tx,ty));
        }
    }
    verts->resize(resX*resY);
    norms->resize(resX*resY);
    geom->setNormalArray(norms);
    geom->setVertexArray(verts);
    geom->setTexCoordArray(0,texarray);
    geode->addDrawable( geom );

    // Set up for dynamic data buffer objects
    geom->setDataVariance( osg::Object::DYNAMIC );
    geom->setUseDisplayList( false );
    geom->setUseVertexBufferObjects( true );
    geom->getOrCreateVertexBufferObject()->setUsage( GL_DYNAMIC_DRAW );

    // Flag state: 2-sided lighting and a texture map.
    {
        osg::StateSet* stateSet( geom->getOrCreateStateSet() );

        osg::LightModel* lm( new osg::LightModel() );
        lm->setTwoSided( true );
        stateSet->setAttributeAndModes( lm );

        const std::string texName( "Images/evilsmiley.png");//fort_mchenry_flag.jpg" );
        osg::Texture2D* tex( new osg::Texture2D(
            osgDB::readImageFile( texName ) ) );
        if( ( tex == NULL ) || ( tex->getImage() == NULL ) )
            osg::notify( osg::WARN ) << "Unable to read texture: \"" << texName << "\"." << std::endl;
        else
        {
            tex->setResizeNonPowerOfTwoHint( false );
            stateSet->setTextureAttributeAndModes( 0, tex );
        }
    }


    // Create the soft body using a Bullet helper function. Note that
    // our update callback will update vertex data from the soft body
    // node data, so it's important that the corners and resolution
    // parameters produce node data that matches the vertex data.
    btSoftBody* softBody( btSoftBodyHelpers::CreatePatch(  bw->getWorldInfo(),
        osgbCollision::asBtVector3( llCorner ),
        osgbCollision::asBtVector3( llCorner + uVec ),
        osgbCollision::asBtVector3( llCorner + vVec ),
        osgbCollision::asBtVector3( llCorner + uVec + vVec ),
        resX, resY, 1+4 , true ) );

        ///retrive index from Bullet faces

    #define FILLINDEX(TYPE)  { osg::DrawElements##TYPE * primset=new osg::DrawElements##TYPE(GL_TRIANGLES);\
    for(int i=0;i<softBody->m_faces.size();i++)\
        for(short ni=0;ni<3;ni++)\
            primset->push_back(softBody->m_faces[i].m_n[ni]-&softBody->m_nodes[0]);\
    geom->addPrimitiveSet(primset);}
    unsigned int numIndices=softBody->m_faces.size()*3;
    if(numIndices<128)          FILLINDEX(UShort)
    else if(numIndices<65535)   FILLINDEX(UByte)
    else                        FILLINDEX(UInt)

    // Configure the soft body for interaction with the wind.
    softBody->getCollisionShape()->setMargin( 0.1 );
    softBody->m_materials[ 0 ]->m_kLST = 0.3;
    softBody->generateBendingConstraints( 2, softBody->m_materials[ 0 ] );
    softBody->m_cfg.kLF = 0.05;
    softBody->m_cfg.kDG = 0.01;
    softBody->m_cfg.piterations = 2;
#if( BT_BULLET_VERSION >= 279 )
    softBody->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSidedLiftDrag;
#else
    // Hm. Not sure how to make the wind blow on older Bullet.
    // This doesn't seem to work.
    softBody->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSided;
#endif
    softBody->setWindVelocity( btVector3( 40., 0., 0. ) );
    softBody->setTotalMass( .1 );
    btTransform m;
    m.setOrigin(btVector3(0.01,0,0));
//softBody->setWorldTransform(m);
  //  bw->addSoftBody( softBody );
geom->setSoftBody(softBody);
//return geom;
   // geom->setUpdateCallback( new MeshUpdater( softBody) );

    return( geode.release() );
}

btSoftRigidDynamicsWorld* initPhysics()
{
    btSoftBodyRigidBodyCollisionConfiguration* collision = new btSoftBodyRigidBodyCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collision );

    btConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface* broadphase = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );


    btSoftRigidDynamicsWorld* dynamicsWorld = new btSoftRigidDynamicsWorld( dispatcher, broadphase, solver, collision );

    btVector3 gravity( 0, 0, -10.17 );
    dynamicsWorld->setGravity( gravity );
    #if 0
    worldInfo.m_gravity = gravity;
    worldInfo.m_broadphase = broadphase;
    worldInfo.air_density = btScalar( 1.2 );
    worldInfo.water_density = 0;
    worldInfo.water_offset = 0;
    worldInfo.water_normal = btVector3( 0, 0, 0 );
    worldInfo.m_sparsesdf.Initialize();
  worldInfo.m_dispatcher = dispatcher;
  #else
       dynamicsWorld->getWorldInfo().m_gravity = gravity;
     dynamicsWorld->getWorldInfo().m_broadphase = broadphase;
     dynamicsWorld->getWorldInfo().air_density = btScalar( 1.2 );
     dynamicsWorld->getWorldInfo().water_density = 0;
     dynamicsWorld->getWorldInfo().water_offset = 0;
     dynamicsWorld->getWorldInfo().water_normal = btVector3( 0, 0, 0 );
     dynamicsWorld->getWorldInfo().m_sparsesdf.Initialize();
   dynamicsWorld->getWorldInfo().m_dispatcher = dispatcher;
  #endif
    return( dynamicsWorld );
}


int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    const bool debugDisplay( arguments.find( "--debug" ) > 0 );

    btSoftRigidDynamicsWorld* bw = initPhysics();
     osgbDynamics::World* root = new osgbDynamics::World;
root->setWorldType(osgbDynamics::World::RIGID_AND_SOFT);
root->setDynamicsWorld(bw);
    osg::Group* launchHandlerAttachPoint = new osg::Group;



    osg::ref_ptr< osg::Node > rootModel( makeFlag( bw ) );
    if( !rootModel.valid() )
    {
        osg::notify( osg::FATAL ) << "mesh: Can't create flag mesh." << std::endl;
        return( 1 );
    }

    root->addChild( rootModel.get() );
 root->addChild( launchHandlerAttachPoint );

//    osg::ref_ptr< osgbInteraction::SaveRestoreHandler > srh = new        osgbInteraction::SaveRestoreHandler;

    // Add ground
    const osg::Vec4 plane( 0., 0., 1., 0. );
    root->addChild( osgbDynamics::generateGroundPlane( plane, bw ) );



    osgbCollision::GLDebugDrawer* dbgDraw( NULL );
    if( debugDisplay )
    {
        dbgDraw = new osgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        bw->setDebugDrawer( dbgDraw );
        root->addChild( dbgDraw->getSceneGraph() );
    }


    osgViewer::Viewer viewer( arguments );
    viewer.addEventHandler( new osgViewer::StatsHandler() );
    //viewer.setUpViewInWindow( 30, 30, 768, 480 );


    /**/   osgDB::writeNodeFile(*root,"fok.osgt");
    root=(osgbDynamics::World*)osgDB::readNodeFile("fok.osgt");
  root->setDebugEnabled(true);

    viewer.setSceneData( root );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( osg::Vec3( 0., -16., 6. ), osg::Vec3( 0., 0., 5. ), osg::Vec3( 0., 0., 1. ) );
    viewer.setCameraManipulator( tb );
    viewer.getCamera()->setClearColor( osg::Vec4( .5, .5, .5, 1. ) );
    viewer.realize();

    // Create the launch handler.
    osgbInteraction::LaunchHandler* lh = new osgbInteraction::LaunchHandler(
        static_cast<btSoftRigidDynamicsWorld*>(root->getDynamicsWorld()), launchHandlerAttachPoint  );
    {
        // Use a custom launch model: Sphere with radius 0.2 (instead of default 1.0).
        osg::Geode* geode = new osg::Geode;
        const double radius( 1 );
        geode->addDrawable( new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(),radius)));//osgwTools::makeGeodesicSphere( radius ) );
        lh->setLaunchModel( geode, new btSphereShape( radius ) );
        lh->setInitialVelocity( 40. );

        viewer.addEventHandler( lh );
    }

   /* srh->setLaunchHandler( lh );
    srh->capture();
    viewer.addEventHandler( srh.get() );*/
    viewer.addEventHandler( new osgbInteraction::DragHandler(
       static_cast<btSoftRigidDynamicsWorld*>(root->getDynamicsWorld()), viewer.getCamera() ) );


    double prevSimTime = 0.;

    return viewer.run();

    while( !viewer.done() )
    {
        if( dbgDraw != NULL )
            dbgDraw->BeginDraw();

        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
       // bw->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;

        if( dbgDraw != NULL )
        {
            bw->debugDrawWorld();
            dbgDraw->EndDraw();
        }

         static_cast<btSoftRigidDynamicsWorld*>(root->getDynamicsWorld())->getWorldInfo().m_sparsesdf.GarbageCollect();

        viewer.frame();
    }

    return( 0 );
}
