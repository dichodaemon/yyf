/***************************************************************************

    file                 : yyf.cpp
    created              : Thu Dec 19 15:29:07 CET 2013
    copyright            : (C) 2002 YU Yufeng

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

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

#include "driver.h"

#define BuffSize 20
#define CarNum 10
static char carNames[CarNum][BuffSize] = {	"yyf_1","yyf_2","yyf_3","yyf_4","yyf_5",
											"yyf_6","yyf_7","yyf_8","yyf_9","yyf_10"};

// for E-track 5 (Oval Tracks) [5 right 1 host 4 inverse]
static float carInitialPoses[CarNum] = {155*5, 155*4, 155*3, 155*2, 155*1, 155*10, 155*9, 155*8, 155*7, 155*6};
static int leadingCarLabel = 5;

// for E-track 5 (Oval Tracks) [9 right 1 host] (host vehicle should be in the last position)
//static float carInitialPoses[CarNum] = {155*9, 155*8, 155*7, 155*6, 155*5, 155*4, 155*3, 155*2, 155*1, 155*0};
//static int leadingCarLabel = 9;

// for Aalborg (Road Tracks)
//static float carInitialPoses[carNum] = {155}
static Driver *driver[CarNum];

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newRace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s);
static int  pitcmd(int index, tCarElt* car, tSituation *s);
static void endRace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 


FILE* fpDebug;

/* 
 * Module entry point  
 */ 
extern "C" int yyf(tModInfo *modInfo)
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

	int i;

	for (i=0; i<CarNum; i++)
	{
		modInfo[i].name    = strdup(carNames[i]);		/* name of the module (short) */
		modInfo[i].desc    = strdup("");	/* description of the module (can be long) */
		modInfo[i].fctInit = InitFuncPt;		/* init function */
		modInfo[i].gfId    = ROB_IDENT;		/* supported framework version */
		modInfo[i].index   = i;
	}
    return 0; 
} 

/* Module interface initialization. */
static int InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    /* create robot instance for index */
	if (index != leadingCarLabel)
    	driver[index] = new Driver(index, 25, (index>=leadingCarLabel) ? true:false, carInitialPoses[index], (index>=leadingCarLabel) ? leadingCarLabel : leadingCarLabel-1);
	else
		driver[index] = new Driver(index, 40, false);

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
    itf->rbNewRace  = newRace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = pitcmd;
    itf->rbEndRace  = endRace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 


/* Called for every track change or new race. */
static void initTrack(int index, tTrack* track, void *carHandle,
                      void **carParmHandle, tSituation *s)
{
    driver[index]->initTrack(track, carHandle, carParmHandle, s);
}

/* Start a new race. */
static void newRace(int index, tCarElt* car, tSituation *s)
{
//	fpDebug = fopen("tmpCarSpeed.txt","w");
    driver[index]->newRace(car, s);
}


/* Drive during race. */
static void drive(int index, tCarElt* car, tSituation *s)
{
    driver[index]->drive(s);
//	fprintf(fpDebug,"%f, %d\n", car->_speed_x, car->_gear);
}


/* Pitstop callback */
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
    return driver[index]->pitCommand(s);
}


/* End of the current race */
static void endRace(int index, tCarElt *car, tSituation *s)
{
    driver[index]->endRace(s);
//	if (fpDebug)	fclose(fpDebug);
}


/* Called before the module is unloaded */
static void shutdown(int index)
{
//	if (fpDebug)	fclose(fpDebug);
    delete driver[index];
}
