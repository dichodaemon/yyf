#include "semaphore.h"
#include <sys/sem.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <iostream>
#include <string.h>
#include <errno.h>

Semaphore::Semaphore( int key )
{
  id_ = semget ( key, 1, IPC_CREAT | 0777 );
  std::cerr << "Created semaphore, return code:" << id_ << std::endl;

  int result = semctl( id_, 0, SETVAL, 1 );
  if ( result != 0 ) {
    std::cerr << "Problem initializing semaphore:" << strerror( errno ) << std::endl;
  }
}

Semaphore::~Semaphore()
{
  struct sembuf ignoredArgument;
  semctl( id_, 1, IPC_RMID, ignoredArgument );
  std::cerr << "Semaphore has been destroyed\n";
}

int 
Semaphore::wait()
{
  struct sembuf parameters;
  parameters.sem_num = 0;
  parameters.sem_op = -1;
  parameters.sem_flg = SEM_UNDO;
  return semop( id_, &parameters, 1 );
}

int 
Semaphore::post()
{
  struct sembuf parameters;
  parameters.sem_num = 0;
  parameters.sem_op = 1;
  parameters.sem_flg = SEM_UNDO;
  return semop( id_, &parameters, 1 );
}
