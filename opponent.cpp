/***************************************************************************

    file                 : opponent.cpp
    created              : Thu Apr 22 01:20:19 CET 2003
    copyright            : (C) 2003 Bernhard Wymann

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "opponent.h"

/* class variables and constants */
tTrack* Opponent::track;
float Opponent::FRONTCOLLDIST = 200.0;	/* [m] distance to check for other cars */
float Opponent::BACKCOLLDIST = 50.0;	/* [m] distance to check for other cars */
float Opponent::LENGTH_MARGIN = 2.0;	/* [m] safety margin */
float Opponent::SIDE_MARGIN = 1.0;		/* [m] safety margin */
float Opponent::MIN_SPEED_DIF = 2.0;	/* [m/s] front car's speed is higher than "my speed + it" */

Opponent::Opponent()
{
}


/* compute speed component parallel to the track */
float Opponent::getSpeed(tCarElt *car)
{
	v2d speed, dir;
	float trackangle = RtTrackSideTgAngleL(&(car->_trkPos));

	speed.x = car->_speed_X;
	speed.y = car->_speed_Y;
	dir.x = cos(trackangle);
	dir.y = sin(trackangle);
	return fabs(speed*dir);
}


/* Compute the length to the start of the segment */
float Opponent::getDistToSegStart()
{
	if (car->_trkPos.seg->type == TR_STR) {
		return car->_trkPos.toStart;
	} else {
		return car->_trkPos.toStart*car->_trkPos.seg->radius;
	}
}


/* Update the values in Opponent this */
void Opponent::update(tSituation *s, Driver *driver)
{
	tCarElt *mycar = driver->getCarPtr();

	/* init state of opponent to ignore */
	state = OPP_IGNORE;

	/* if the car is out of the simulation ignore it */
	if (car->_state & RM_CAR_STATE_NO_SIMU) {
		return;
	}

	/* updating distance along the middle */
	float oppToStart = car->_trkPos.seg->lgfromstart + getDistToSegStart();
	distance = oppToStart - mycar->_distFromStartLine;
	while (distance > track->length/2.0)
	{
		distance -= track->length;
	}
	while (distance < -track->length/2.0)
	{
		distance += track->length;
	}

	/* update speed in track direction */
	speed = Opponent::getSpeed(car);
	float cosa = speed/sqrt(car->_speed_X*car->_speed_X + car->_speed_Y*car->_speed_Y);
	float alpha = acos(cosa);
	width = car->_dimension_x*fabs(sin(alpha)) + car->_dimension_y*fabs(cosa);
	float SIDECOLLDIST = MIN(car->_dimension_x, mycar->_dimension_x);

	/* is opponent in relevant range -50..200 m */
	if (distance > -BACKCOLLDIST && distance < FRONTCOLLDIST)
	{
		/* is opponent in front and slower */
		if (distance > SIDECOLLDIST && fabs(speed) < fabs(driver->getSpeed()) + MIN_SPEED_DIF)
		{
			catchdist = driver->getSpeed()*distance/(driver->getSpeed() - speed);
			state |= OPP_FRONT;
			distance -= MAX(car->_dimension_x, mycar->_dimension_x);
			distance -= LENGTH_MARGIN;
			float cardist = car->_trkPos.toMiddle - mycar->_trkPos.toMiddle;
			sidedist = cardist;
			cardist = fabs(cardist) - fabs(width/2.0) - mycar->_dimension_y/2.0;
			if (cardist < SIDE_MARGIN)
				state |= OPP_COLL;
		} else
		/* is opponent behind and faster */
		if (distance < -SIDECOLLDIST && fabs(speed) < fabs(driver->getSpeed()) + MIN_SPEED_DIF)
		{
			catchdist = driver->getSpeed()*distance/(speed - driver->getSpeed());
			state |= OPP_BACK;
			distance += MAX(car->_dimension_x, mycar->_dimension_x);
			distance += LENGTH_MARGIN;
			float cardist = car->_trkPos.toMiddle - mycar->_trkPos.toMiddle;
			sidedist = cardist;
			cardist = fabs(cardist) - fabs(width/2.0) - mycar->_dimension_y/2.0;
			if (cardist < SIDE_MARGIN)
				state |= OPP_COLL;
		} else
		/* is opponent aside */
		if (distance > -SIDECOLLDIST &&	distance < SIDECOLLDIST)
		{
			sidedist = car->_trkPos.toMiddle - mycar->_trkPos.toMiddle;
			state |= OPP_SIDE;
		}
	}
}


/* Initialize the list of opponents */
Opponents::Opponents(tSituation *s, Driver *driver)
{
	opponent = new Opponent[s->_ncars - 1];
	int i, j = 0;
	for (i = 0; i < s->_ncars; i++) {
		if (s->cars[i] != driver->getCarPtr()) {
			opponent[j].setCarPtr(s->cars[i]);
			j++;
		}
	}
	Opponent::setTrackPtr(driver->getTrackPtr());
	nopponents = s->_ncars - 1;
}


Opponents::~Opponents()
{
	delete [] opponent;
}


/* Updates all the Opponent instances */
void Opponents::update(tSituation *s, Driver *driver)
{
	int i;
	for (i = 0; i < s->_ncars - 1; i++) {
		opponent[i].update(s, driver);
	}
}
