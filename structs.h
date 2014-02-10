#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <vector>
#include <stdint.h>

const int MAX_OBSTACLE_NUMBER = 100;

// car parameters (save to the file)
struct CarParam
{
	float maxRPM;
	float wheelRadius;
	int nGear;
	std::vector <float> gearRatio;
	int gearOffset;
	float width;
	float length;
	float steerLock;
};

// track parameters (save to the file)
struct TrackSeg
{
	int id;
	float length;
	float width;
	float curvature;
	float angle;
	float distFromStart;	// the distance from the first segment to the start point
	float allowedSpeed;		// Cheat from the simulator
};

struct TrackParam
{
	float length;
	float width;
	int nSeg;
	std::vector <TrackSeg> segs;
};

// command parameters (send online)
struct Command {
	float steering;
	float acceleration;
	float brake;
	int gear;
};

// status parameters about the host vehicle (send online)
/*struct Status {
	int gear;
	float rpm;  
	float speed;
	float yaw;
	float x;
	float y;
};
*/

struct Status {
  float rpm;
  int gear;
  float speed;
  float yaw;
  float x;
  float y;
};

struct Obstacle {
  uint8_t id;
  float x;
  float y;
  float theta;
  float vX;
  float vY;
  float width;
  float length;
};

typedef std::vector<Obstacle> Obstacles;

struct Buffer {
  Command command;
  Status status;
  uint8_t nObstacles;
  Obstacle obstacles[100];
};

#endif //STRUCTS_H_

