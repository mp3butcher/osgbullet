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
#include <osgbDynamics/World.h>
#include <osgbInteraction/LaunchHandler.h>

#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2010 Robert Osfield
 *
 * This application is open source and may be redistributed and/or modified
 * freely and without restriction, both in commercial and non commercial applications,
 * as long as this copyright notice is maintained.
 *
 * This application is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>

#include <osg/Switch>
#include <osg/Types>
#include <osgText/Text>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>

#include <osgGA/Device>

#include <iostream>


int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is the standard OpenSceneGraph example which loads and visualises 3d models.");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");
    arguments.getApplicationUsage()->addCommandLineOption("--image <filename>","Load an image and render it on a quad");
    arguments.getApplicationUsage()->addCommandLineOption("--dem <filename>","Load an image/DEM and render it on a HeightField");
    arguments.getApplicationUsage()->addCommandLineOption("--login <url> <username> <password>","Provide authentication information for http file access.");
    arguments.getApplicationUsage()->addCommandLineOption("-p <filename>","Play specified camera path animation file, previously saved with 'z' key.");
    arguments.getApplicationUsage()->addCommandLineOption("--speed <factor>","Speed factor for animation playing (1 == normal speed).");
    arguments.getApplicationUsage()->addCommandLineOption("--device <device-name>","add named device to the viewer");

    osgViewer::Viewer viewer(arguments);

    unsigned int helpType = 0;
    if ((helpType = arguments.readHelpType()))
    {
        arguments.getApplicationUsage()->write(std::cout, helpType);
        return 1;
    }

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }

    /*if (arguments.argc()<=1)
    {
        arguments.getApplicationUsage()->write(std::cout,osg::ApplicationUsage::COMMAND_LINE_OPTION);
        return 1;
    }
*/
    std::string url, username, password;
    while(arguments.read("--login",url, username, password))
    {
        if (!osgDB::Registry::instance()->getAuthenticationMap())
        {
            osgDB::Registry::instance()->setAuthenticationMap(new osgDB::AuthenticationMap);
            osgDB::Registry::instance()->getAuthenticationMap()->addAuthenticationDetails(
                url,
                new osgDB::AuthenticationDetails(username, password)
            );
        }
    }

    std::string device;
    while(arguments.read("--device", device))
    {
        osg::ref_ptr<osgGA::Device> dev = osgDB::readRefFile<osgGA::Device>(device);
        if (dev.valid())
        {
            viewer.addDevice(dev);
        }
    }

    // set up the camera manipulators.
    {
        osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

        keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
        keyswitchManipulator->addMatrixManipulator( '2', "Flight", new osgGA::FlightManipulator() );
        keyswitchManipulator->addMatrixManipulator( '3', "Drive", new osgGA::DriveManipulator() );
        keyswitchManipulator->addMatrixManipulator( '4', "Terrain", new osgGA::TerrainManipulator() );
        keyswitchManipulator->addMatrixManipulator( '5', "Orbit", new osgGA::OrbitManipulator() );
        keyswitchManipulator->addMatrixManipulator( '6', "FirstPerson", new osgGA::FirstPersonManipulator() );
        keyswitchManipulator->addMatrixManipulator( '7', "Spherical", new osgGA::SphericalManipulator() );

        std::string pathfile;
        double animationSpeed = 1.0;
        while(arguments.read("--speed",animationSpeed) ) {}
        char keyForAnimationPath = '8';
        while (arguments.read("-p",pathfile))
        {
            osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
            if (apm || !apm->valid())
            {
                apm->setTimeScale(animationSpeed);

                unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
                keyswitchManipulator->addMatrixManipulator( keyForAnimationPath, "Path", apm );
                keyswitchManipulator->selectMatrixManipulator(num);
                ++keyForAnimationPath;
            }
        }

        viewer.setCameraManipulator( keyswitchManipulator.get() );
    }

    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the thread model handler
    viewer.addEventHandler(new osgViewer::ThreadingHandler);

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    // add the help handler
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    // add the record camera path handler
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);

    // add the LOD Scale handler
    viewer.addEventHandler(new osgViewer::LODScaleHandler);

    // add the screen capture handler
    viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);

    // load the data
    osg::ref_ptr<osg::Node> loadedModel = osgDB::readRefNodeFiles(arguments);
  /*  if (!loadedModel)
    {
        std::cout << arguments.getApplicationName() <<": No data loaded" << std::endl;
        return 1;
    }*/

    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }


    // optimize the scene graph, remove redundant nodes and state etc.


osg::Group* root=(osg::Group*)osgDB::readNodeFile("/home/pascal/SRC/osgCastWizard2/bin/bullettest.osgb");

osgbDynamics::World* w=(osgbDynamics::World*)root->getChild(0);
//w->setDebugEnabled(true);
    viewer.setSceneData( root );
  osgUtil::Optimizer optimizer;
    optimizer.optimize(root);


   /* osgDB::writeNodeFile(*root.get(),"fok.osgt");



    btRigidBody* hack=((osgbDynamics::RigidBody*)root->getChild(root->getNumChildren()-1))->getRigidBody();
   ///not serialized
    hack-> setCollisionFlags( boxBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
    hack->setActivationState( DISABLE_DEACTIVATION );


  osg::Group* launchHandlerAttachPoint = new osg::Group;
    w->addChild( launchHandlerAttachPoint );


 osgbInteraction::LaunchHandler* lh = new osgbInteraction::LaunchHandler();
     //lh->setWorld(root);
     //   lh->setAttachPoint( root );
    {
        // Use a custom launch model: Sphere with radius 0.2 (instead of default 1.0).
        osg::Geode* geode = new osg::Geode;
        const double radius( 0.2 );
        geode->addDrawable( new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(),radius)));//osgwTools::makeGeodesicSphere( radius ) );
        lh->setLaunchModel( geode, new btSphereShape( radius ) );
        lh->setInitialVelocity( 40. );

        //viewer.addEventHandler( lh );
    }*/
    viewer.realize();
     viewer.run();
  /*  while( !viewer.done() )
    {
        currSimTime = viewer.getFrameStamp()->getSimulationTime();
  //    dynamicsWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;
        viewer.frame();
    }*/

    return( 0 );
}

