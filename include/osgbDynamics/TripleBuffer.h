/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009-2011 by Kenneth Mark Bryden
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


#ifndef __OSGBDYNAMICS_TRIPLEBUFFER_H__
#define __OSGBDYNAMICS_TRIPLEBUFFER_H__ 1

#include <osgbDynamics/Export.h>
#include <OpenThreads/Mutex>
#include <iostream>
#include <stdlib.h>


namespace osgbDynamics {


/** \class TripleBuffer TripleBuffer.h <osgbDynamics/TripleBuffer.h>
\brief TBD

A generic triple buffer mechanism. Allows the read
thread to always access the latest complete set of updated
data without blocking. Allows the write thread to always 
switch to an available buffer to write the next set of data
without blocking.

Multiple heterogeneous chunks of data may be stored in buffers
by using reserve() to specify a data size and retrieve back the
unsigned int index into the buffer(s). This index is a count of
least-resolvable machine units (usually bytes).

In a typical usage case, each MotionState has a matrix
stored in the triple buffer object. During a physics sim step, the
physics sim thread calls beginWrite, access the matrices for current
rigid body location and updates them, then calls endWrite. Concurrently,
the rendering update thread calls beginRead, uses the data to update
matrices in the scene graph, then calls endRead.
*/
class OSGBDYNAMICS_EXPORT TripleBuffer
{
public:
    TripleBuffer( unsigned int initialSize=8192 );
    ~TripleBuffer();

    void resize( unsigned int size );
    unsigned int reserve( unsigned int size, char* data=NULL );

    char* beginWrite();
    char* writeAddress();
    void endWrite();

    char* beginRead();
    char* readAddress();
    void endRead();

    void debugDump( const std::string& msg, std::ostream& oStr ) const;

protected:
    unsigned int _currentSize;
    unsigned int _nextFree;

    typedef enum {
        INVALID = 0,
        UPDATED = 1,
        READ = 2,
        WRITE = 3
    } BufferStatus;
    BufferStatus _status[ 3 ];
    char* _buf[ 3 ];

    char* _writeAddress;
    char* _readAddress;

    int get( BufferStatus status );
    void reallocate( unsigned int index, unsigned int size );

    OpenThreads::Mutex _lock;
};


// osgbDynamics
}

// __OSGBDYNAMICS_TRIPLEBUFFER_H__
#endif