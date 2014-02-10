#ifndef FACADE_H_
#define FACADE_H_

#include "structs.h"
#include "semaphore.h"


class Facade {
public:
  Facade( int key = 5555 );
  ~Facade();
  Command getCommand();
  void setCommand( const Command & command );
  void setStatus( const Status & status );
  void setObstacles( const Obstacles & obstacles );
  void setBuffer( const Buffer & buffer);

private:
  int segmentId_;
  int segmentSize_;
  Buffer * buffer_;
  Semaphore semaphore_;
};

#endif //FACADE_H_

