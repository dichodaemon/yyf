#include "facade.h"
#include "structs.h"
#include <sys/shm.h>
#include <sys/stat.h>
#include <iostream>

Facade::Facade( int key ) 
  : segmentSize_( sizeof( Buffer ) ),
    semaphore_( key )
{
  segmentId_ = shmget(
    key, segmentSize_, IPC_CREAT | S_IRUSR | S_IWUSR | S_IROTH | S_IWOTH
  );
  buffer_ = (Buffer*) shmat( segmentId_, 0, 0 );
}

Facade::~Facade()
{
  shmctl( segmentId_, IPC_RMID, 0 );
  shmdt( buffer_ );
}

Command 
Facade::getCommand() 
{
  Command result;
  semaphore_.wait();
  result = buffer_->command;
  semaphore_.post();
  return result;
}

void 
Facade::setCommand( const Command & command ) 
{
  semaphore_.wait();
  buffer_->command = command;
  semaphore_.post();
}

void 
Facade::setStatus( const Status & status ) 
{
  semaphore_.wait();
  buffer_->status = status;
  semaphore_.post();
}

void
Facade::setObstacles( const Obstacles & obstacles )
{
  semaphore_.wait();
  buffer_->nObstacles = obstacles.size();
  for ( int i = 0; i < buffer_->nObstacles; ++i ) {
    buffer_->obstacles[i] = obstacles[i];
  }
  semaphore_.post();
}

void
Facade::setBuffer( const Buffer & buffer )
{
  semaphore_.wait();
  buffer_->command = buffer.command;
  buffer_->status = buffer.status;
  buffer_->nObstacles = buffer.nObstacles;
  for ( int i = 0; i < buffer_->nObstacles; ++i ) {
    buffer_->obstacles[i] = buffer.obstacles[i];
  }
  semaphore_.post();
}


