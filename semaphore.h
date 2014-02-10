#ifndef SEMAPHORE_H_
#define SEMAPHORE_H_


class Semaphore
{
public:
  Semaphore( int key = 5555 );
  ~Semaphore();
  int wait();
  int post();
private:
  int id_;
};

#endif //SEMAPHORE_H_

