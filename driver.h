/***************************************************************************

    file                 : driver.h
    created              : Thu Dec 20 01:20:19 CET 2002
    copyright            : (C) 2002 Bernhard Wymann

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "linalg.h"
#include "opponent.h"

class Opponents;
class Opponent;

class Driver {
	public:
		//Driver(int index);
		//Driver(int index, float max_speed);
		Driver(int index, float max_speed = FLT_MAX, bool inverse_driving = false, int initial_pose=-1, int leading_car=-1);
		~Driver();

		/* callback functions called from TORCS */
		void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
		void newRace(tCarElt* car, tSituation *s);
		void drive(tSituation *s);
		int pitCommand(tSituation *s);
		void endRace(tSituation *s);
		tCarElt *getCarPtr() { return car; }
		tTrack *getTrackPtr() { return track; }
		float getSpeed() { return speed; }

	private:

		float MaxSpeed;
		bool InverseDriving;
		bool InverseYawFinished;
		float InitialPose;
		bool PoseInitialized;
		int LeadingCar;
		int inverse_stage;
		int vel_keep_time;

		/* utility functions */
		bool inverseYaw();

		bool isStuck();
		void update(tSituation *s);
		float getAllowedSpeed(tTrackSeg *segment);
		float getAccel();
		float getDistToSegEnd();
		float getDistToSegStart();
		float getBrake();
		int getGear();
		float getSteer();
		v2d getTargetPoint();
		float getOvertakeOffset();

		float filterABS(float brake);
		float filterTCL(float accel);
		float filterTCL_RWD();
		float filterTCL_FWD();
		float filterTCL_4WD();
		void initTCLfilter();
		float filterTrk(float accel);
		float filterBColl(float brake);
		float filterSColl(float steer);

		void initCa();
		void initCw();

		/* per robot global data */
		int stuck;
		float trackangle;
		float angle;
		float mass;			/* mass of car + fuel */
		float speed;		/* speed in track direction */
		tCarElt *car;		/* pointer to tCarElt struct */
		float myoffset;		/* overtake offset sideways */
		Opponents *opponents;
        Opponent *opponent;

		/* data that should stay constant after first initialization */
		int MAX_UNSTUCK_COUNT;
		int INDEX;
		float CARMASS;		/* mass of the car only */
		float CA;			/* aerodynamic downforce coefficient */
		float CW;			/* aerodynamic drag coefficient */
		float (Driver::*GET_DRIVEN_WHEEL_SPEED)();

		/* class constants */
		static const float MAX_UNSTUCK_ANGLE;
		static const float UNSTUCK_TIME_LIMIT;
		static const float MAX_UNSTUCK_SPEED;
		static const float MIN_UNSTUCK_DIST;
		static const float G;
		static const float FULL_ACCEL_MARGIN;
		static const float SHIFT;
		static const float SHIFT_MARGIN;
		static const float ABS_SLIP;
		static const float ABS_MINSPEED;
		static const float TCL_SLIP;
		static const float TCL_MINSPEED;
		static const float LOOKAHEAD_CONST;
		static const float LOOKAHEAD_FACTOR;
		static const float WIDTHDIV;
		static const float SIDECOLL_MARGIN;
		static const float BORDER_OVERTAKE_MARGIN;
		static const float OVERTAKE_OFFSET_INC;
		static const float MIN_TIME_FOR_BRAKE_START;
		static const float MIN_DIST_FOR_SPEED_LIMIT;
		static const int   TIME_FOR_VEL_CHANGE;
		static const int   ZIG_ZAG_TIME;
		static const float MIN_DIST_BETWEEN_CARS;		
		// yyf Test for Kalman filter 20140204
		static const int MAX_COUNT_FOR_CHANGE;
		static const int INDEX_FOR_CHANGE;
		int countForChange;
		// yyf Test for velocity
		static const int INDEX_FOR_VELOCITY;
		static const float ACCEL_START_POINT;
		static const float ACCEL_STOP_POINT;
		static const float BRAKE_START_POINT;
		static const float BRAKE_STOP_POINT;
		static const int MAX_STEPS_FOR_VELOCITY;
		static const float MAX_SPEED_FOR_TEST;
		int step;
		bool brakeBeforeAccel;		
		bool accelStart;
		bool brakeStart;
		FILE* logFile;
		void VelocityTest();


		/* track variables */
		tTrack* track;
};

#endif // _DRIVER_H_

