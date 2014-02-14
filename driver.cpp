/***************************************************************************

    file                 : driver.cpp
    created              : Thu Dec 20 01:21:49 CET 2002
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

#include "driver.h"

#define BT_SECT_PRIV "bt private"
#define BT_ATT_FUELPERLAP "fuelperlap"

const float Driver::MIN_DIST_BETWEEN_CARS = 1.0;			/* [m] */
const float Driver::MIN_TIME_FOR_BRAKE_START = 0.2;			/* [s] */
const float Driver::MAX_UNSTUCK_ANGLE = 30.0/180.0*PI;		/* [radians] */
const float Driver::UNSTUCK_TIME_LIMIT = 2.0;				/* [s] */
const float Driver::MAX_UNSTUCK_SPEED = 5.0;				/* [m/s] */
const float Driver::MIN_UNSTUCK_DIST = 3.0;					/* [m] */
const float Driver::G = 9.81;								/* [m/(s*s)] */
const float Driver::FULL_ACCEL_MARGIN = 1.0;				/* [m/s] */
const float Driver::SHIFT = 0.9;							/* [-] (% of rpmredline) */
const float Driver::SHIFT_MARGIN = 4.0;						/* [m/s] */
const float Driver::ABS_SLIP = 0.9;							/* [-] range [0.95..0.3] */
const float Driver::ABS_MINSPEED = 5.0;						/* [m/s] */
const float Driver::TCL_SLIP = 0.9;							/* [-] range [0.95..0.3] */
const float Driver::TCL_MINSPEED = 3.0;						/* [m/s] */
const float Driver::LOOKAHEAD_CONST = 5.0;					/* [m] */
const float Driver::LOOKAHEAD_FACTOR = 0.33;				/* [-] */
const float Driver::WIDTHDIV = 3.0;							/* [-] */
const float Driver::SIDECOLL_MARGIN = 2.0;					/* [m] */
const float Driver::BORDER_OVERTAKE_MARGIN = 0.5;			/* [m] */
const float Driver::OVERTAKE_OFFSET_INC = 0.05;				/* [m/timestep] */
const float Driver::MIN_DIST_FOR_SPEED_LIMIT = 80.0;		/* [m] */
const int   Driver::TIME_FOR_VEL_CHANGE = 1000;
const int 	Driver::ZIG_ZAG_TIME = 6;

// yyf Test for Kalman filter 20140204 [the car will drive like a snake]
const int Driver::INDEX_FOR_CHANGE = -1;
const int Driver::MAX_COUNT_FOR_CHANGE = 200;

// yyf Test for velocity [for accel, brake, slow_down_with_accel data]
const int Driver::INDEX_FOR_VELOCITY = -1;
const int Driver::MAX_STEPS_FOR_ACCEL = 10;
const int Driver::MAX_STEPS_FOR_BRAKE = 10;
const int Driver::MAX_STEPS_FOR_BA = 3;
const float Driver::ACCEL_START_POINT = 715;
const float Driver::ACCEL_STOP_POINT = 1300; //900
const float Driver::BRAKE_START_POINT = 1530;
const float Driver::BRAKE_STOP_POINT = 95;
const float Driver::BRAKE_ACCEL_START_POINT = 715;
const float Driver::BRAKE_ACCEL_STOP_POINT = 1300;
const float Driver::MAX_SPEED_FOR_TEST = 33.4;

// yyf Test for overtaking [the overtaking car (host vehicle like human_ros)]
const int Driver::INDEX_FOR_OVERTAKING = 5;	// for [5 right 1 host 4 inverse]
const int Driver::WAIT_FRAME_FOR_OVERTAKING = 4000;
const float Driver::MIN_DIST_FOR_OVERTAKING = 20;
const float Driver::MIN_SPEED_FOR_OVERTAKING = 30.0;


// initial_pose  is for senario generation, every car will go to the initial pose and stop
// when "index == leading_car" drive, the environmental car drive
Driver::Driver(int index, float max_speed, bool inverse_driving, int initial_pose, int leading_car)
{
	INDEX = index;
	MaxSpeed = max_speed;
	InverseDriving = inverse_driving;
	InverseYawFinished = false;
	vel_keep_time = TIME_FOR_VEL_CHANGE;
	inverse_stage = -1;
	InitialPose = initial_pose;
	PoseInitialized = false;
	LeadingCar = leading_car;
}

Driver::~Driver()
{
    delete opponents;
}


/* Called for every track change or new race. */
void Driver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s)
{
	track = t;
	*carParmHandle = NULL;
}


/* Start a new race. */
void Driver::newRace(tCarElt* car, tSituation *s)
{
	MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/RCM_MAX_DT_ROBOTS);
	stuck = 0;
	myoffset = 0.0;
	this->car = car;
	CARMASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);
	initCa();
	initCw();
	initTCLfilter();

	/* initialize the list of opponents */
	opponents = new Opponents(s, this);
	opponent = opponents->getOpponentPtr();

	//yyf Test for Kalman filter 20140204
	countForChange=0;
	// yyf Test for velocity
	brakeBeforeAccel = false;		
	accelStart = false;
	brakeStart = false;
	brakeAccelStart = false;
	accelStep = 0;
	brakeStep = 0;
	brakeAccelStep = 0;
	roundTestVelocity = 0;

	// yyf Test for overtaking
	wait_frame_now = 0;
}


bool Driver::inverseYaw()
{
	const float InverseSpeedLimit = 5;
	const float InverseYawThresh = 0.2;
	const float StopSpeed = 0.5;
	int calculated_stage;
	float ang = this->angle;
	float speed = sqrt(car->_speed_x*car->_speed_x + car->_speed_y*car->_speed_y);

	while (ang<-PI)	ang += 2*PI;
	while (ang>=PI)	ang -= 2*PI;
	ang = fabs(ang);
	
	calculated_stage = 	MAX( ZIG_ZAG_TIME-(int)(ang/PI*ZIG_ZAG_TIME) - 1, 0);

	if (inverse_stage < 0)			// Initialize
		inverse_stage = calculated_stage;

	if ( inverse_stage %2 == 0 )
	{
//		printf("stage:%d\t right %f\n", inverse_stage, ang);
		car->ctrl.steer = -1;
		car->ctrl.gear = 1;
	}else
	{
//		printf("stage:%d\t left %f\n", inverse_stage, ang);
		car->ctrl.steer = 1;
		car->ctrl.gear = -1;
	}

	if (speed > InverseSpeedLimit)
	{
		car->ctrl.accelCmd = 0;
		car->ctrl.brakeCmd = 0.5;
 	}else
	{
		car->ctrl.accelCmd = 1;
		car->ctrl.brakeCmd = 0;
	}

	if (inverse_stage < calculated_stage)		// wait until the car totally stop
	{
		car->ctrl.accelCmd = 0;
		car->ctrl.brakeCmd = 0.5;
		if (speed <= StopSpeed)
		{
			inverse_stage = calculated_stage;
			if (inverse_stage >= ZIG_ZAG_TIME - 1)
				return true;
		}
	}
	return false;
}

/* Drive during race. */
void Driver::drive(tSituation *s)
{
	update(s);
	memset(&car->ctrl, 0, sizeof(tCarCtrl));

	if (InverseDriving && !InverseYawFinished)
	{
		InverseYawFinished = inverseYaw();
		return;
	}

	if (isStuck()) {
		car->ctrl.steer = -angle / car->_steerLock;
		car->ctrl.gear = -1; // reverse gear
		car->ctrl.accelCmd = 0.5; // 30% accelerator pedal
		car->ctrl.brakeCmd = 0.0; // no brakes
	} else {
		car->ctrl.steer = filterSColl(getSteer());
		car->ctrl.gear = getGear();
		
		if (InitialPose>=0 && !PoseInitialized && fabs( InitialPose - car->_distFromStartLine)<20 )
		{
			car->ctrl.gear = 0;
			car->ctrl.brakeCmd =1.0; // brake when reaching the InitialPose
			if (fabs(opponent[LeadingCar].getSpeed()) > 5.0)
				PoseInitialized = true;
		}
		else
			car->ctrl.brakeCmd = filterABS(filterBColl(getBrake()));
		if (car->ctrl.brakeCmd == 0.0) {
			car->ctrl.accelCmd = filterTCL(filterTrk(getAccel()));
		} else {
			car->ctrl.accelCmd = 0.0;
		}
	}
	
	// yyf Test for velocity
	if (INDEX==INDEX_FOR_VELOCITY)
		VelocityTest();

	// yyf Test for overtaking
	// wait until the scenario is done
	if (INDEX==INDEX_FOR_OVERTAKING)
	{
		wait_frame_now++;
		if (wait_frame_now<WAIT_FRAME_FOR_OVERTAKING)
		{
			car->ctrl.accelCmd = 0.0;
			car->ctrl.brakeCmd = 1.0;
			car->ctrl.gear = 0;
		}else
			SendMessages(INDEX, car, s);
		// printf("%d\n",wait_frame_now);
	}
}


/* Set pitstop commands. */
int Driver::pitCommand(tSituation *s)
{
	return ROB_PIT_IM; /* return immediately */
}


/* End of the current race */
void Driver::endRace(tSituation *s)
{
}


/***************************************************************************
 *
 * utility functions
 *
***************************************************************************/


/* Compute the allowed speed on a segment */
float Driver::getAllowedSpeed(tTrackSeg *segment)
{
	float length = 0.0;
	float arc = 0.0;
	float speed_limit = MaxSpeed;
	float mu,r;
	tTrackSeg *s = segment;
	tTrackSeg *sOrigin = s;

	s = segment;
	if (s->type == TR_STR)
	{
		return MaxSpeed;
    }
	sOrigin = s;
	arc = 0;
	while (s->type == sOrigin->type && arc < PI/2.0)
	{
		arc += s->arc;
		if (!InverseDriving)
			s = s->next;
		else
			s = s->prev;
	}
	// arc /= PI/2.0;
	mu = sOrigin->surface->kFriction;

	r = sOrigin->radius;
	if ( (sOrigin->type == TR_RGT && InverseDriving) || (sOrigin->type != TR_RGT && !InverseDriving) )
		r += sOrigin->width/4;
	else
		r -= sOrigin->width/4;
	
	/* // for higher speed limit
	float w = sOrigin->width/4;
	if ( r*(1-cos(arc))<= w )
	{
		return MaxSpeed;
	}
	r = r + w*(2*r+w) / (2*(r*(1-cos(arc))-w));
	*/
	return MIN( MaxSpeed, sqrt((mu*G*r)/(1.0 - MIN(1.0, r*CA*mu/mass))));
}


/* Compute the length to the end of the segment */
float Driver::getDistToSegEnd()
{
	if (car->_trkPos.seg->type == TR_STR) {
		return car->_trkPos.seg->length - car->_trkPos.toStart;
	} else {
		return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
	}
}

float Driver::getDistToSegStart()
{
	if (car->_trkPos.seg->type == TR_STR)
	{
		return car->_trkPos.toStart;
	}else
	{
		return car->_trkPos.toStart*car->_trkPos.seg->radius;
	}
}


/* Compute fitting acceleration */
float Driver::getAccel()
{
	float allowedspeed = getAllowedSpeed(car->_trkPos.seg);
	float gr = car->_gearRatio[car->_gear + car->_gearOffset];
	float rm = car->_enginerpmRedLine;
	if (allowedspeed > car->_speed_x + FULL_ACCEL_MARGIN) {
		return 1.0;
	} else {
		return allowedspeed/car->_wheelRadius(REAR_RGT)*gr /rm;
	}
}


float Driver::getBrake()
{
	tTrackSeg *segptr = car->_trkPos.seg;
	float currentspeedsqr = car->_speed_x*car->_speed_x;
	float mu = segptr->surface->kFriction;
	float maxlookaheaddist = currentspeedsqr/(2.0*mu*G) * 1.5;
	float lookaheaddist = getDistToSegEnd();

	float allowedspeed = getAllowedSpeed(segptr);
	float allowedspeedsqr;
	float brakedist;

	if (allowedspeed < car->_speed_x)
	{
		//printf("brake here\n");
		return 1.0;
	}

	// Pose Initializing
	if (InitialPose>=0 && !PoseInitialized)
	{
		// if (INDEX==0)
		//	printf("index: %d\t Pose: %f Car: %f\n",INDEX, InitialPose, car->_distFromStartLine);
		allowedspeed = 0;
		lookaheaddist = fabs(car->_distFromStartLine - InitialPose);
		if (lookaheaddist > track->length/2.0)
			lookaheaddist = track->length - lookaheaddist;

		allowedspeed = 0;
		float allowedspeedsqr = allowedspeed*allowedspeed;
		brakedist = mass*(currentspeedsqr - allowedspeedsqr) / (2.0*(mu*G*mass + allowedspeedsqr*(CA*mu + CW)));
		if (brakedist * 1.2 > lookaheaddist) {
			return 1.0;
		}else if (brakedist * 1.4 > lookaheaddist)
			return 0.5;
		else if (brakedist *1.6 > lookaheaddist)
			return 0.2;
	}

	// normal brake control
	lookaheaddist = 0;
	while (lookaheaddist < maxlookaheaddist) {
		allowedspeed = getAllowedSpeed(segptr);
		if (allowedspeed < car->_speed_x) {
			allowedspeedsqr = allowedspeed*allowedspeed;
			brakedist = mass*(currentspeedsqr - allowedspeedsqr) / (2.0*(mu*G*mass + allowedspeedsqr*(CA*mu + CW)));
			if (brakedist * 1.2 > lookaheaddist) {
				return 1.0;
			}else if (brakedist * 1.4 > lookaheaddist)
				return 0.5;
			else if (brakedist * 1.6 > lookaheaddist)
				return 0.2;
		}
		lookaheaddist += segptr->length;
		
		if (!InverseDriving)
			segptr = segptr->next;
		else
			segptr = segptr->prev;
	}
	
	return 0.0;
}


/* Compute gear */
int Driver::getGear()
{
	if (car->_gear <= 0)
		return 1;
	float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
	float omega = car->_enginerpmRedLine/gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*SHIFT < car->_speed_x) {
		return car->_gear + 1;
	} else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = car->_enginerpmRedLine/gr_down;
		if (car->_gear > 1 && omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) {
			return car->_gear - 1;
		}
	}
	return car->_gear;
}


/* compute steer value */
float Driver::getSteer()
{
	float targetAngle;
	v2d target = getTargetPoint();

	targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
	targetAngle -= car->_yaw;
	NORM_PI_PI(targetAngle);
	return targetAngle / car->_steerLock;
}

/* compute target point for steering */
v2d Driver::getTargetPoint()
{
	tTrackSeg *seg = car->_trkPos.seg;
	float lookahead, length, offset;

	lookahead = LOOKAHEAD_CONST + car->_speed_x*LOOKAHEAD_FACTOR;

	if (!InverseDriving)
	{
		length = getDistToSegEnd();
		while (length < lookahead) {
			seg = seg->next;
			length += seg->length;
		}
		length = lookahead - length + seg->length;
		offset = -seg->width/4;
	}else
	{
		length = getDistToSegStart();
		while (length < lookahead)
		{
			seg = seg->prev;
			length += seg->length;
		}
		length = length - lookahead;
		offset = seg->width/4;
	}

	// yyf Test for overtaking
	if (INDEX == INDEX_FOR_OVERTAKING)
	{
		if (!InverseDriving)
			offset += getOvertakeOffset();
		else
			offset -= getOvertakeOffset();
	}

	// yyf Test for Kalman filter
	if (INDEX == INDEX_FOR_CHANGE)
	{
		countForChange++;
		if ( (countForChange / MAX_COUNT_FOR_CHANGE) % 2 == 0)
			offset = seg->width/4;
		else
			offset = -seg->width/4;
		// printf("%f\n",offset);
	}

	v2d s;
	s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0;
	s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0;

	if ( seg->type == TR_STR) {
		v2d d, n;
		n.x = (seg->vertex[TR_EL].x - seg->vertex[TR_ER].x)/seg->length;
		n.y = (seg->vertex[TR_EL].y - seg->vertex[TR_ER].y)/seg->length;
		n.normalize();
		d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x)/seg->length;
		d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y)/seg->length;
		return s + d*length + offset*n;
	} else {
		v2d c, n;
		c.x = seg->center.x;
		c.y = seg->center.y;
		float arc = length/seg->radius;
		float arcsign = (seg->type == TR_RGT) ? -1.0 : 1.0;
		arc = arc*arcsign;
		s = s.rotate(c, arc);
		n = c - s;
		n.normalize();
		return s + arcsign*offset*n;
	}
}



/* Compute an offset to the target point */
float Driver::getOvertakeOffset()
{
	int i;
	float catchdist;
	float mincatchdist = FLT_MAX;
	float mindist = FLT_MAX;
	Opponent *o = NULL;
	
	// the segments allowed overtaking
	bool overtakeAllowed = true;
	float overtakeDistance = 100;

	tTrackSeg *segptr = car->_trkPos.seg;
	float lookaheaddist = 0;
	while (lookaheaddist < overtakeDistance)
	{
		if (getAllowedSpeed(segptr) < MIN_SPEED_FOR_OVERTAKING)
		{
			overtakeAllowed = false;
			break;
		}
		lookaheaddist += segptr->length;
		if (!InverseDriving)
			segptr = segptr->next;
		else
			segptr = segptr->prev;

	}

	for (i = 0; i < opponents->getNOpponents(); i++)
	{
		if ( (!InverseDriving && (opponent[i].getState() & OPP_FRONT))
			||(InverseDriving && (opponent[i].getState() & OPP_BACK)) )
		{
			catchdist = opponent[i].getCatchDist();
			if (catchdist < mincatchdist && opponent[i].getDistance() < mindist)
			{
				mincatchdist = catchdist;
				mindist = opponent[i].getDistance();
				o = &opponent[i];
			}
		}
	}



	/*
	if (o != NULL && mindist<=MIN_DIST_FOR_OVERTAKING)
	{
		float w = o->getCarPtr()->_trkPos.seg->width/WIDTHDIV - BORDER_OVERTAKE_MARGIN;
		float otm = o->getCarPtr()->_trkPos.toMiddle;
		if (otm > 0.0 && myoffset > -w) myoffset -= OVERTAKE_OFFSET_INC;
		else if (otm < 0.0 && myoffset < w) myoffset += OVERTAKE_OFFSET_INC;
	}else
	{
		if (myoffset > OVERTAKE_OFFSET_INC) myoffset -= OVERTAKE_OFFSET_INC;
		else if (myoffset < -OVERTAKE_OFFSET_INC) myoffset += OVERTAKE_OFFSET_INC;
		else myoffset = 0.0;
	}*/

	if (overtakeAllowed && o != NULL && mindist<=MIN_DIST_FOR_OVERTAKING)
	{
		//printf("%f\n",myoffset);
		float w = o->getCarPtr()->_trkPos.seg->width/WIDTHDIV;
		if (myoffset < w)
			myoffset += OVERTAKE_OFFSET_INC;
		else
			myoffset -= OVERTAKE_OFFSET_INC;			
	}else
	{
		if (myoffset > OVERTAKE_OFFSET_INC)
			myoffset -= OVERTAKE_OFFSET_INC;
		else if (myoffset < -OVERTAKE_OFFSET_INC)
				myoffset += OVERTAKE_OFFSET_INC;
			else
				myoffset = 0.0;
	}

	return myoffset;
}


/* Update my private data every timestep */
void Driver::update(tSituation *s)
{
	trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
	if (!InverseDriving)
		angle = trackangle - car->_yaw;
	else
		angle = trackangle + PI - car->_yaw;
	NORM_PI_PI(angle);
	mass = CARMASS + car->_fuel;
	speed = Opponent::getSpeed(car);
	opponents->update(s, this);
}


/* Check if I'm stuck */
bool Driver::isStuck()
{
	if (fabs(angle) > MAX_UNSTUCK_ANGLE &&
		car->_speed_x < MAX_UNSTUCK_SPEED &&
		fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
		if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle*angle < 0.0) {
			return true;
		} else {
			stuck++;
			return false;
		}
	} else {
		stuck = 0;
		return false;
	}
}


/* Compute aerodynamic downforce coefficient CA */
void Driver::initCa()
{
	char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
	float rearwingarea = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*) NULL, 0.0);
	float rearwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*) NULL, 0.0);
	float wingca = 1.23*rearwingarea*sin(rearwingangle);

	float cl = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char*) NULL, 0.0) +
			   GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char*) NULL, 0.0);
	float h = 0.0;
	int i;
	for (i = 0; i < 4; i++)
		h += GfParmGetNum(car->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char*) NULL, 0.20);
	h*= 1.5; h = h*h; h = h*h; h = 2.0 * exp(-3.0*h);
	CA = h*cl + 4.0*wingca;
}


/* Compute aerodynamic drag coefficient CW */
void Driver::initCw()
{
	float cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char*) NULL, 0.0);
	float frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*) NULL, 0.0);
	CW = 0.645*cx*frontarea;
}


/* Brake filter for collision avoidance */
float Driver::filterBColl(float brake)
{
	float currentspeedsqr = car->_speed_x*car->_speed_x;
	float mu = car->_trkPos.seg->surface->kFriction;
	float cm = mu*G*mass;
	float ca = CA*mu + CW;
	int i;

	for (i = 0; i < opponents->getNOpponents(); i++)
	{
		if (!InverseDriving)
		{
			if ( (opponent[i].getState() & OPP_COLL) && (opponent[i].getState() & OPP_FRONT))
			{
				float allowedspeedsqr = opponent[i].getSpeed();
				allowedspeedsqr *= allowedspeedsqr;
				float brakedist = mass*(currentspeedsqr - allowedspeedsqr) / (2.0*(cm + allowedspeedsqr*ca));
				brakedist += MIN_TIME_FOR_BRAKE_START * car->_speed_x;
				if (brakedist + MIN_DIST_BETWEEN_CARS > fabs(opponent[i].getDistance()))
				{
					return 1.0;
				}
			}
		}else
		{
			//printf("\nindex: %d\t",INDEX);
			//if (INDEX==6 && i==6)	printf("%f\n",opponent[i].getDistance());
			if ( (opponent[i].getState() & OPP_COLL) && (opponent[i].getState() & OPP_BACK))
			{
				//printf("COLL\t");
				float allowedspeedsqr = opponent[i].getSpeed();
				allowedspeedsqr *= allowedspeedsqr;
				float brakedist = mass*(currentspeedsqr - allowedspeedsqr) / (2.0*(cm + allowedspeedsqr*ca));
				brakedist += MIN_TIME_FOR_BRAKE_START * car->_speed_x;
				//printf("allow: %f\tself %f\tdist%f",allowedspeedsqr, car->_speed_x, brakedist);
				if (brakedist + MIN_DIST_BETWEEN_CARS > fabs(opponent[i].getDistance()))
				{
					return 1.0;
				}
			}			
		}
	}
	return brake;
}


/* Steer filter for collision avoidance */
float Driver::filterSColl(float steer)
{
	int i;
	float sidedist = 0.0, fsidedist = 0.0, minsidedist = FLT_MAX;
	Opponent *o = NULL;

	/* get the index of the nearest car (o) */
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if (opponent[i].getState() & OPP_SIDE) {
			sidedist = opponent[i].getSideDist();
			fsidedist = fabs(sidedist);
			if (fsidedist < minsidedist) {
				minsidedist = fsidedist;
				o = &opponent[i];
			}
		}
	}

	/* if there is another car handle the situation */
	if (o != NULL) {
		float d = fsidedist - o->getWidth();
		/* near enough */
		if (d < SIDECOLL_MARGIN) {
			/* compute angle between cars */
			tCarElt *ocar = o->getCarPtr();
			float diffangle = ocar->_yaw - car->_yaw;
			
			if (cos(diffangle)<0)	diffangle += PI;// deal with the reverse car
			NORM_PI_PI(diffangle);
			const float c = SIDECOLL_MARGIN/2.0;
			d = d - c;
			if (d < 0.0) d = 0.0;
			float psteer = diffangle/car->_steerLock;
			return steer*(d/c) + 2.0*psteer*(1.0-d/c);
		}
	}
	return steer;
}


/* Antilocking filter for brakes */
float Driver::filterABS(float brake)
{
	if (brake<0)
		return -brake;
	if (car->_speed_x < ABS_MINSPEED) return brake;
	int i;
	float slip = 0.0;
	for (i = 0; i < 4; i++) {
		slip += car->_wheelSpinVel(i) * car->_wheelRadius(i) / car->_speed_x;
	}
	slip = slip/4.0;
	if (slip < ABS_SLIP) brake = brake*slip;
	return brake;
}


/* TCL filter for accelerator pedal */
float Driver::filterTCL(float accel)
{
	if (car->_speed_x < TCL_MINSPEED) return accel;
	float slip = car->_speed_x/(this->*GET_DRIVEN_WHEEL_SPEED)();
	if (slip < TCL_SLIP) {
		accel = 0.0;
	}
	return accel;
}


/* Traction Control (TCL) setup */
void Driver::initTCLfilter()
{
	const char *traintype = GfParmGetStr(car->_carHandle, SECT_DRIVETRAIN, PRM_TYPE, VAL_TRANS_RWD);
	if (strcmp(traintype, VAL_TRANS_RWD) == 0) {
		GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_RWD;
	} else if (strcmp(traintype, VAL_TRANS_FWD) == 0) {
		GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_FWD;
	} else if (strcmp(traintype, VAL_TRANS_4WD) == 0) {
		GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_4WD;
	}
}


/* TCL filter plugin for rear wheel driven cars */
float Driver::filterTCL_RWD()
{
	return (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
			car->_wheelRadius(REAR_LFT) / 2.0;
}


/* TCL filter plugin for front wheel driven cars */
float Driver::filterTCL_FWD()
{
	return (car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
			car->_wheelRadius(FRNT_LFT) / 2.0;
}


/* TCL filter plugin for all wheel driven cars */
float Driver::filterTCL_4WD()
{
	return (car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
			car->_wheelRadius(FRNT_LFT) / 4.0 +
		   (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
			car->_wheelRadius(REAR_LFT) / 4.0;
}


/* Hold car on the track */
float Driver::filterTrk(float accel)
{
	tTrackSeg* seg = car->_trkPos.seg;

	if (car->_speed_x < MAX_UNSTUCK_SPEED) return accel;
	
	if (seg->type == TR_STR) {
		float tm = fabs(car->_trkPos.toMiddle);
		float w = seg->width/WIDTHDIV;
		if (tm > w) return 0.0; else return accel;
	} else {
		float sign = (seg->type == TR_RGT) ? -1 : 1;
		if (car->_trkPos.toMiddle*sign > 0.0) {
			return accel;
		} else {
			float tm = fabs(car->_trkPos.toMiddle);
			float w = seg->width/WIDTHDIV;
			if (tm > w) return 0.0; else return accel;
		}
	}
}

// yyf Test for velocity
// the clock is not correct
float Driver::GetTimeFloat()
{
//	printf("%f %d\n", ((float)clock()) , CLOCKS_PER_SEC);
	return ((float)clock()) / CLOCKS_PER_SEC;
}


void Driver::VelocityTest()
{
	char FileDir[] = "/home/emotion/yuyu/INRIA_documents/Accel_Brake_Para/";
	MaxSpeed = MAX_SPEED_FOR_TEST;
	float EPS = 5.0;
	float min_speed = 0.5;
	float speed = sqrt(car->_speed_x*car->_speed_x + car->_speed_y*car->_speed_y);
	char FileName[255];

	//car->ctrl.accelCmd = 0.2;
	//car->ctrl.brakeCmd = 0.0; 
	//FILE* fp = fopen("timeCheck.txt","a");
	//fprintf(fp,"%f\t%f\t%f\n", GetTimeFloat(), speed, car->_distFromStartLine);
	//fclose(fp);
	
	/*
	if ( abs(speed - MaxSpeed) < 1.0)
	{
		brakeAccelStart = true;
	}
	if (brakeAccelStart == true)
	{
		car->ctrl.gear = 2;
		car->ctrl.accelCmd = 0.2;
		car->ctrl.brakeCmd = 0.0;
	}
	*/

	
	if (accelStart)	// accel from v=0
	{
		car->ctrl.accelCmd = accelStep*1.0/MAX_STEPS_FOR_ACCEL;
		car->ctrl.brakeCmd = 0.0;
		if (logFile!=NULL)
			fprintf(logFile, "%f\t%f\n", speed, car->ctrl.accelCmd);		
		if ( speed >= MAX_SPEED_FOR_TEST || abs(car->_distFromStartLine - ACCEL_STOP_POINT) < EPS )
		{
			accelStart = false;
			SafeFclose(logFile);
		}
		return;
	}else if (brakeStart) // brake
	{
		car->ctrl.accelCmd = 0.0;
		car->ctrl.brakeCmd = brakeStep*1.0/MAX_STEPS_FOR_BRAKE;
		if (logFile!=NULL)
			fprintf(logFile, "%f\t%f\n", speed, -car->ctrl.brakeCmd);
		if ( speed <= min_speed || abs(car->_distFromStartLine - BRAKE_STOP_POINT) < EPS )
		{
			brakeStart = false;
			SafeFclose(logFile);
		}
		return;
	}else if (brakeAccelStart) // brake by accel from max_speed
	{
		car->ctrl.accelCmd = (brakeAccelStep-1)*1.0/MAX_STEPS_FOR_ACCEL;
		car->ctrl.brakeCmd = 0.0;
		if (logFile!=NULL)
			fprintf(logFile, "%f\t%f\n", speed, car->ctrl.accelCmd);		
		if ( speed <= min_speed || abs(car->_distFromStartLine - BRAKE_STOP_POINT) < EPS )
		{
			brakeAccelStart = false;
			SafeFclose(logFile);
		}
		return;
	}else if (brakeBeforeAccel) // brake to v=0
	{
		car->ctrl.accelCmd = 0.0;
		car->ctrl.brakeCmd = 1.0;
		if ( speed <= min_speed)
		{
			accelStep++;
			accelStep = (accelStep-1)%MAX_STEPS_FOR_ACCEL + 1;	
			if (accelStep==1)
			{
				brakeStep = 0;
				roundTestVelocity++;
				brakeAccelStep = 0;
			}				
			sprintf(FileName,"%s%s_accel_%02d0(%02d).txt", FileDir, car->_carName, accelStep, roundTestVelocity);
			logFile = fopen(FileName,"w");
			brakeBeforeAccel = false;
			accelStart = true;
		}
		return;
	}else
	{
		if ( abs(car->_distFromStartLine - ACCEL_START_POINT) < EPS )
		{
			if (accelStep == brakeAccelStep + 1 && brakeAccelStep < MAX_STEPS_FOR_BA)
			{
				brakeAccelStep++;
				brakeAccelStep = (brakeAccelStep-1)%MAX_STEPS_FOR_BA + 1;
				sprintf(FileName,"%s%s_brakeAccel_%02d0(%02d).txt", FileDir, car->_carName, brakeAccelStep-1, roundTestVelocity);
				logFile = fopen(FileName,"w");
				brakeAccelStart = true;
			}
			else
			{
				brakeBeforeAccel = true;
			}
		}else if ( abs(car->_distFromStartLine - BRAKE_START_POINT) < EPS )
		{
			brakeStep++;
			brakeStep = (brakeStep-1)%MAX_STEPS_FOR_BRAKE + 1;
			sprintf(FileName,"%s%s_brake_%02d0(%02d).txt",FileDir, car->_carName, brakeStep, roundTestVelocity);
			logFile = fopen(FileName,"w");
			brakeStart = true;
		}
	}
}

bool Driver::SafeFclose(FILE* fp)
{
	if (fp!=NULL)
		fclose(fp);
	return true;
}
