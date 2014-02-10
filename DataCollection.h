#ifndef DATACOLLECTION_H
#define DATACOLLECTION_H

#ifdef _WIN32
#include <windows.h>
#define isnan _isnan
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <plib/js.h>

#include <tgfclient.h>
#include <portability.h>

#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>
#include <string.h>

#include "facade.h"
#include "structs.h"

bool InitCarData(tCarElt* car);
bool SaveCarData(const CarParam& car, char* FileName);
bool LoadCarData(CarParam& car, char* FileName);

bool InitTrackData(tTrack* track);
bool SaveTrackData(const TrackParam& track, char* FileName);
bool LoadTrackData(TrackParam& track, char* FileName);

float computeCurvature( tTrackSeg * segment );
bool SendMessages(int index, tCarElt* car, tSituation *s);
Command GetCommandData();
float GetAllowedSpeed(tTrackSeg *segment);

#endif

