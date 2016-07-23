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
#ifndef OLDSAVRESTOREMODEL
///written by J.Valentin
///use experimental osgdb_serializer (exploiting bullet serialization model)



#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/AnimationPath>
#include <osg/PolygonMode>
#include <osg/PolygonOffset>

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <iostream>

#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/RigidBodyAnimation.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbDynamics/World.h>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>

osg::AnimationPath * createAnimationPath( const osg::Vec3 & center,
                                          float radius,
                                          double looptime )
{
    /* set up the animation path */
    osg::AnimationPath * animationPath = new osg::AnimationPath;

    animationPath->setLoopMode( osg::AnimationPath::LOOP );

    int numSamples = 40;
    float yaw = 0.0f;
    float yaw_delta = 2.0f * osg::PI / ( ( float )numSamples - 1.0f );
    float roll = osg::inDegrees( 30.0f );

    double time = 0.0f;
    double time_delta = looptime / ( double )numSamples;
    for( int i = 0; i < numSamples; ++i )
    {
        osg::Vec3 position( center + osg::Vec3( sinf( yaw ) * radius, cosf( yaw ) * radius, 0.0f ) );
        osg::Quat rotation( osg::Quat( roll, osg::Vec3( 0.0, 1.0, 0.0 ) ) * osg::Quat( -( yaw + osg::inDegrees( 90.0f ) ), osg::Vec3( 0.0, 0.0, 1.0 ) ) );

        animationPath->insert( time, osg::AnimationPath::ControlPoint( position, rotation ) );

        yaw += yaw_delta;
        time += time_delta;
    }
    return( animationPath );
}

osg::MatrixTransform * createOSGBox( osg::Vec3 size )
{
    osg::Box * box = new osg::Box();

    box->setHalfLengths( size );

    osg::ShapeDrawable * shape = new osg::ShapeDrawable( box );

    osg::Geode * geode = new osg::Geode();
    geode->addDrawable( shape );

    osg::MatrixTransform * transform = new osg::MatrixTransform();
    transform->addChild( geode );

    return( transform );
}

btRigidBody * createBTBox( osg::MatrixTransform * box,
                          osg::Vec3 center )
{
    btCollisionShape * collision = osgbCollision::btBoxCollisionShapeFromOSG( box );

    osgbDynamics::MotionState * motion = new osgbDynamics::MotionState();
    motion->setTransform( box );
    motion->setParentTransform( osg::Matrix::translate( center ) );

    btScalar mass( 0.0 );
    btVector3 inertia( 0, 0, 0 );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, collision, inertia );
    btRigidBody * body = new btRigidBody( rb );

    return( body );
}

btDynamicsWorld * initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -10 ) );

    return( dynamicsWorld );
}

float randomFloatInRange( std::pair< float, float > range =
                            std::pair< float, float >( -10, 10 ) )
{
    return( range.first + (range.second-range.first)*rand()/static_cast< float >( RAND_MAX ) );
}

btVector3 randomBVec3InRange( std::pair< btVector3, btVector3 > range =
                        std::pair< btVector3, btVector3 >(
                            btVector3( -1, -1, -1),
                            btVector3(  1,  1,  1) ) )
{
    return(
        btVector3(
            randomFloatInRange(
                std::pair< float, float >( range.first[0], range.second[0] ) ),
            randomFloatInRange(
                std::pair< float, float >( range.first[1], range.second[1] ) ),
            randomFloatInRange(
                std::pair< float, float >( range.first[2], range.second[2] ) ) ) );
}


/* \cond */
class GliderUpdateCallback : public osg::NodeCallback
{
public:
    GliderUpdateCallback( btRigidBody * body )
        : body_( body )
        , basetime_( 0.0 )
        {}
    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        double now = nv->getFrameStamp()->getSimulationTime();
        if ( ( now - basetime_ ) > 5.5 )
        {
            basetime_ = now;
            btVector3 punch = randomBVec3InRange(
                std::pair< btVector3, btVector3 >(
                btVector3( -10, -10, -.5 ), btVector3(  10,  10, .5 ) ) );
            osg::notify( osg::NOTICE ) << "punch!"
                << punch[0] << " "
                << punch[1] << " "
                << punch[2] << std::endl;
            body_->setLinearVelocity( punch );
            body_->setAngularVelocity( randomBVec3InRange() );
        }
        traverse( node, nv );
    }
private:
    btRigidBody *   body_;
    double          basetime_;
};
/* \endcond */


osg::MatrixTransform * createModel( btDynamicsWorld * dynamicsWorld )
{
/*
 * BEGIN: Create physics object code.
 *  OSG CODE
 */
    osg::ref_ptr< osg::MatrixTransform > node;
	const std::string fileName( "glider.osg" );
    osg::ref_ptr< osg::Node > nodeDB = osgDB::readNodeFile( fileName );
	if( !nodeDB.valid() )
	{
		osg::notify( osg::FATAL ) << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the OSG sample data directory." << std::endl;
		exit( 0 );
	}

    if( ( node = dynamic_cast< osg::MatrixTransform * >( nodeDB.get() ) ) == NULL )
    {
        node = new osg::MatrixTransform;
        node->addChild( nodeDB.get() );
    }

    /*  osgBullet code */
    osgbDynamics::MotionState * motion = new osgbDynamics::MotionState;
    motion->setTransform( node.get() );
  //ConvexHullCollisionShape outperform  btCollisionShape * collision = osgbCollision::btConvexTriMeshCollisionShapeFromOSG( node.get() );
       btCollisionShape * collision =  osgbCollision::btConvexHullCollisionShapeFromOSG( node.get() );
    // Create an OSG representation of the Bullet shape and attach it.
    // This is mainly for debugging.
    osg::Node* debugNode = osgbCollision::osgNodeFromBtCollisionShape( collision );
    node->addChild( debugNode );

    // Set debug node state.
    osg::StateSet* state = debugNode->getOrCreateStateSet();
    osg::PolygonMode* pm = new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
    state->setAttributeAndModes( pm );
    osg::PolygonOffset* po = new osg::PolygonOffset( -1, -1 );
    state->setAttributeAndModes( po );
    state->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    /*  BULLET CODE */
    btTransform bodyTransform;
    bodyTransform.setIdentity();
    bodyTransform.setOrigin( btVector3( 0, 0, 5 ) );
    motion->setWorldTransform( bodyTransform );

    btScalar mass( 1.0 );
    btVector3 inertia;
    collision->calculateLocalInertia( mass, inertia );
    btRigidBody::btRigidBodyConstructionInfo rbinfo( mass, motion, collision, inertia );
    btRigidBody * body = new btRigidBody( rbinfo );
    body->setLinearVelocity( btVector3( -5, -1, 0 ) );
    body->setAngularVelocity( btVector3( 1, 0, 0 ) );

      osgbDynamics:: RigidBody* gliderrig=new osgbDynamics::RigidBody();
    gliderrig->setRigidBody( body);
    node->addUpdateCallback(gliderrig);
    //dynamicsWorld->addRigidBody( body );

    // kick thing around from time to time.
    node->addUpdateCallback( new GliderUpdateCallback( body ) );

    return( node.release() );
}

int main( int argc,
          char * argv[] )
{
    osg::ArgumentParser arguments( &argc, argv );
    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );

    osgGA::TrackballManipulator * tb = new osgGA::TrackballManipulator();

    tb->setHomePosition( osg::Vec3( 5, -12, 12 ),
                        osg::Vec3( -7, 0, -10 ),
                        osg::Vec3( 0, 0, 1 ) );
    viewer.setCameraManipulator( tb );

    osg::ref_ptr< osg::Group > root = new osg::Group;

    osgDB::getDataFilePathList().push_back( "C:\\OpenSceneGraph\\Data" );

    btDynamicsWorld * dynamicsWorld = initPhysics();

    root->addChild( createModel( dynamicsWorld ) );

    /* BEGIN: Create environment boxes */
    osg::MatrixTransform * ground;
    btRigidBody * groundBody;

   osgbDynamics:: World * osgbtworld=new osgbDynamics::World;
    osgbtworld->setDynamicsWorld(dynamicsWorld);
    root->addUpdateCallback(osgbtworld);

    float thin = .01;
    ground = createOSGBox( osg::Vec3( 10, 10, thin ) );
    root->addChild( ground );
    groundBody = createBTBox( ground, osg::Vec3( 0, 0, -10 ) );
    {
  osgbDynamics:: RigidBody* groundBodyrig=new osgbDynamics::RigidBody();
    groundBodyrig->setRigidBody( groundBody);
    ground->addUpdateCallback(groundBodyrig);
    //dynamicsWorld->addRigidBody( groundBody );
}
    ground = createOSGBox( osg::Vec3( 10, thin, 5 ) );
    root->addChild( ground );
    groundBody = createBTBox( ground, osg::Vec3( 0, 10, -5 ) );
{
  osgbDynamics:: RigidBody* groundBodyrig=new osgbDynamics::RigidBody();
    groundBodyrig->setRigidBody( groundBody);
    ground->addUpdateCallback(groundBodyrig);
    //dynamicsWorld->addRigidBody( groundBody );
}

    ground = createOSGBox( osg::Vec3( 10, thin, 5 ) );
    root->addChild( ground );
    groundBody = createBTBox( ground, osg::Vec3( 0, -10, -5 ) );
 {
  osgbDynamics:: RigidBody* groundBodyrig=new osgbDynamics::RigidBody();
    groundBodyrig->setRigidBody( groundBody);
    ground->addUpdateCallback(groundBodyrig);
    //dynamicsWorld->addRigidBody( groundBody );
}

    ground = createOSGBox( osg::Vec3( thin, 10, 5 ) );
    root->addChild( ground );
    groundBody = createBTBox( ground, osg::Vec3( 10, 0, -5 ) );
{
  osgbDynamics:: RigidBody* groundBodyrig=new osgbDynamics::RigidBody();
    groundBodyrig->setRigidBody( groundBody);
    ground->addUpdateCallback(groundBodyrig);
    //dynamicsWorld->addRigidBody( groundBody );
}

    ground = createOSGBox( osg::Vec3( thin, 10, 5 ) );
    root->addChild( ground );
    groundBody = createBTBox( ground, osg::Vec3( -10, 0, -5 ) );
{
  osgbDynamics:: RigidBody* groundBodyrig=new osgbDynamics::RigidBody();
    groundBodyrig->setRigidBody( groundBody);
    ground->addUpdateCallback(groundBodyrig);
    //dynamicsWorld->addRigidBody( groundBody );
}
    /* END: Create environment boxes */

    /* BEGIN: Create animated box */
    /* OSG Code */
    osg::MatrixTransform * box = createOSGBox( osg::Vec3( .3, .3, .3 ) );
    osg::AnimationPathCallback * apc = new osg::AnimationPathCallback( createAnimationPath( osg::Vec3( 0, 0, -9.25 ), 9.4, 6 ), 0, 1 );
    box->addUpdateCallback( apc );
    root->addChild( box );

    /* Bullet Code */
    btRigidBody * boxBody = createBTBox( box, osg::Vec3( -9, -3, -9 ) );
    boxBody->setCollisionFlags( boxBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
    boxBody->setActivationState( DISABLE_DEACTIVATION );
  {
  osgbDynamics:: RigidBody* boxBodyrig=new osgbDynamics::RigidBody();
    boxBodyrig->setRigidBody( boxBody);
    ground->addUpdateCallback(boxBodyrig);
    //dynamicsWorld->addRigidBody( boxBody );
}

    /* osgBullet Code */
    osgbCollision::RefBulletObject< btRigidBody >* boxRigid =
        new osgbCollision::RefBulletObject< btRigidBody >( boxBody );
    box->setUserData( boxRigid );

    osgbDynamics::RigidBodyAnimation * rba = new osgbDynamics::RigidBodyAnimation;
    apc->setNestedCallback( rba );
    /* END: Create animated box */

    // bonus geometry
    root->addChild( osgDB::readNodeFiles( arguments ) );

       osgbCollision::GLDebugDrawer* dbgDraw = new osgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        dynamicsWorld->setDebugDrawer( dbgDraw );
        root->addChild( dbgDraw->getSceneGraph() );

    osgDB::writeNodeFile(*root.get(),"fok.osgt");
    root=(osg::Group*)osgDB::readNodeFile("fok.osgt");



    viewer.setSceneData( root.get() );
    viewer.realize();


   /* viewer.run();*/
    while( !viewer.done() )
    {

            dbgDraw->BeginDraw();

        dynamicsWorld->debugDrawWorld();
            dbgDraw->EndDraw();
                 viewer.frame();
    }

    return( 0 );
}



#else
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>

#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/CreationRecord.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Utils.h>

/#include <osgbInteraction/SaveRestoreHandler.h>
#include <osgbInteraction/DragHandler.h>

#include <osgwTools/InsertRemove.h>
#include <osgwTools/FindNamedNode.h>

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <string>
#include <map>



btDiscreteDynamicsWorld* initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDiscreteDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -9.81 ) );

    return( dynamicsWorld );
}


int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    const bool debugDisplay( arguments.find( "--debug" ) > 0 );

    std::string restoreFileName;
    if( arguments.read( "--restore", restoreFileName ) )
    {
        if( osgDB::findDataFile( restoreFileName ).empty() )
        {
            osg::notify( osg::FATAL ) << "Can't find restore file: \"" << restoreFileName << "\"." << std::endl;
            return( 1 );
        }
    }

    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    osg::ref_ptr< osg::Group > root = new osg::Group;

    const std::string sceneFileName( "saverestore-scene.osg" );
    osg::Node* scene = osgDB::readNodeFile( sceneFileName );
    if( scene == NULL )
    {
        osg::notify( osg::FATAL ) << "saverestore: Can't load data file \"" << sceneFileName << "\"." << std::endl;
        return( 1 );
    }
    root->addChild( scene );

    osg::ref_ptr< osgbInteraction::SaveRestoreHandler > srh = new osgbInteraction::SaveRestoreHandler;

    // Restore the saved PhysicsState.
    if( !( restoreFileName.empty() ) )
        srh->restore( restoreFileName );

    // Find all nodes with names containing "-body". Turn each into a rigid body.
    osgwTools::FindNamedNode fnn( "-body" );
    fnn.setMatchMethod( osgwTools::FindNamedNode::CONTAINS );
    scene->accept( fnn );

    osgwTools::FindNamedNode::NodeAndPathList::const_iterator it;
    for( it = fnn._napl.begin(); it != fnn._napl.end(); it++ )
    {
        // Get the root node of the subgraph that we will make into a rigid body.
        // Also, get its NodePath and local to world transform.
        osg::Node* node = it->first;
        osg::NodePath np = it->second;
        osg::Matrix xform = osg::computeLocalToWorld( np );

        // Rigid bodies need to be rooted at an AbsoluteModelTransform.
        osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform;
        amt->setDataVariance( osg::Object::DYNAMIC );
        osgwTools::insertAbove( node, amt );

        // Manually insert the AMT above the node in the NodePath. Kind of ugly.
        np[ np.size() - 1 ] = amt;
        np.resize( np.size() + 1);
        np[ np.size() - 1 ] = node;

        osg::ref_ptr< osgbDynamics::CreationRecord > cr;
        osgbDynamics::PhysicsData* pd;
        if( restoreFileName.empty() )
        {
            // Not restoring. Configure the CreationRecord.
            cr = new osgbDynamics::CreationRecord;
            cr->_sceneGraph = amt;
            cr->_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
            cr->setCenterOfMass( node->getBound().center() );
            cr->_parentTransform = xform;
            cr->_mass = 1.f;
            cr->_scale = xform.getScale();
            cr->_restitution = .5f;

            srh->add( it->first->getName(), cr.get() );
        }
        else
        {
            // Restoring.
            // Get the CreationRecord from the SaveRestoreHandler, which
            // contains our restored PhysicsData for this node.
            pd = srh->getPhysicsData( it->first->getName() );
            cr = pd->_cr;
            // _sceneGraph is a live address and can't be saved/restored, so set it manually.
            cr->_sceneGraph = amt;
        }

        btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );
        rb->setActivationState( DISABLE_DEACTIVATION );

        if( !( restoreFileName.empty() ) )
        {
            // Now that the rigid body has been created, set its saved transform and linear / angular velocities.
            rb->setWorldTransform( osgbCollision::asBtTransform( pd->_bodyWorldTransform ) );
            rb->setLinearVelocity( osgbCollision::asBtVector3( pd->_linearVelocity ) );
            rb->setAngularVelocity( osgbCollision::asBtVector3( pd->_angularVelocity ) );
        }

        // Required for DragHandler default behavior.
        amt->setUserData( new osgbCollision::RefRigidBody( rb ) );

        bulletWorld->addRigidBody( rb );

        srh->add( it->first->getName(), rb );
    }

    // Add ground
    const osg::Vec4 plane( 0., 0., 1., 0. );
    root->addChild( osgbDynamics::generateGroundPlane( plane, bulletWorld ) );


    osgbCollision::GLDebugDrawer* dbgDraw( NULL );
    if( debugDisplay )
    {
        dbgDraw = new osgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        bulletWorld->setDebugDrawer( dbgDraw );
        root->addChild( dbgDraw->getSceneGraph() );
    }


    osgViewer::Viewer viewer( arguments );
    viewer.setUpViewInWindow( 30, 30, 768, 480 );
    viewer.setSceneData( root.get() );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    //tb->setHomePosition( osg::Vec3( 0., -8., 2. ), osg::Vec3( 0., 0., 1. ), osg::Vec3( 0., 0., 1. ) );
    viewer.setCameraManipulator( tb );
    viewer.getCamera()->setClearColor( osg::Vec4( .5, .5, .5, 1. ) );

    srh->capture();
    viewer.addEventHandler( srh.get() );

    osgViewer::Viewer::Cameras cams;
    viewer.getCameras( cams );
    osg::ref_ptr< osgbInteraction::DragHandler > dh =
        new osgbInteraction::DragHandler( bulletWorld, cams[ 0 ] );
    viewer.addEventHandler( dh.get() );

    viewer.realize();
    double prevSimTime = 0.;
    while( !viewer.done() )
    {
        if( dbgDraw != NULL )
            dbgDraw->BeginDraw();

        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bulletWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;

        if( dbgDraw != NULL )
        {
            bulletWorld->debugDrawWorld();
            dbgDraw->EndDraw();
        }

        viewer.frame();
    }

    return( 0 );
}


/** \page saverestoreexample Save and Restore Example

Demonstrates saving and restoring osgBullet data to/from disk.

Use the --debug command line option to enable debug collision object display.

\section saverestorecontrols UI Controls

\li Delete: Reset the physics simulation to its initial state.
\li ctrl-leftmouse: Select and drag an object.

*/
#endif
