/*  TORCSLink, an interface between TORCS and MATLAB/Simulink
	Copyright(C) 2014 Owen McAree
	Copyright(C) 2017 Erwin Lopez

	This program is free software : you can redistribute it and / or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.If not, see <http://www.gnu.org/licenses/>.
*/

#include <ctime>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "TORCSLink.h"
#include "sensors.h"
#include "SimpleParser.h"
#include "CarControl.h"
#include "ObstacleSensors.h"

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newRace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);
static int InitFuncPt(int index, void *pt);
static void endRace(int index, tCarElt *car, tSituation *s);

static tTrack	*curTrack;

double __SENSORS_RANGE__;
#define __FOCUS_RANGE__ 200

/*** Noise definitions ***/
#define __NOISE_STD__ 0.1
#define __OPP_NOISE_STD__ 0.02
#define __FOCUS_NOISE_STD__ 0.01

static Sensors *trackSens[N_VEHICLES];
static ObstacleSensors *oppSens[N_VEHICLES];
static Sensors *focusSens[N_VEHICLES];//ML
static double trackSensAngle[N_VEHICLES][19];

/* Module entry point */
extern "C" int matlabUoB(tModInfo *modInfo) {
	
	/* Set up shared memory */
	if (tlInitSharedMemory() != 0) {
		printf("matlabUoB robots: Error initialising shared memory!\n");
	}
	
	/* Clear all structures */
	memset(modInfo, 0, N_VEHICLES*sizeof(tModInfo));

	for (int i = 0; i < N_VEHICLES; i++) {
		char name[64];
		sprintf(name, "matlabUoB %d", i);
		modInfo[i].name    = strdup(name);
		modInfo[i].desc	   = strdup(name);
		modInfo[i].fctInit = InitFuncPt;
		modInfo[i].gfId    = ROB_IDENT;	
		modInfo[i].index   = i;
	}
	printf("matlabUoB robots: Initialised!\n");
	return 0;
}


/* Module interface initialization */
static int InitFuncPt(int index, void *pt) {
	tRobotItf *itf = (tRobotItf *)pt;

	itf->rbNewTrack = initTrack;	/* Give the robot the track view called */
	itf->rbNewRace  = newRace;		/* Start a new race */
	itf->rbDrive    = drive;		/* Drive during race */
	itf->rbPitCmd   = NULL;			/* Pit commands */
	itf->rbEndRace  = endRace;		/* End of the current race */
	itf->rbShutdown = shutdown;		/* Called before the module is unloaded */
	itf->index      = index;		/* Index used if multiple interfaces */
	return 0;
}


/* Called for every track change or new race */
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) {
	curTrack = track;
	
	/* Don't load any particular car set up */
	*carParmHandle = NULL;
}

/* Start a new race  */
static void newRace(int index, tCarElt* car, tSituation *sit) {
	
	//Set sensor range
	/*if (strcmp(getVersion(), "2009") == 0)
	{
		__SENSORS_RANGE__ = 100;
		printf("*****2009*****\n");
	}
	else if (strcmp(getVersion(), "2010") == 0 || strcmp(getVersion(), "2011") == 0)*/
	__SENSORS_RANGE__ = 200;
	/*else
	{
		printf("%s is not a recognized version", getVersion());
		exit(0);
	}*/
	//if (SimpleParser::parse(initStr, std::string("init"), trackSensAngle[index], 19) == false)
	//{
		//for (int i = 0; i < 19; ++i) {
		//	trackSensAngle[index][i] = -90 + 10 * i;
		//}
	//}
	if (TRACK_SENSOR_AGR == 0) {
		float a[19] = {-90, -75, -60, -45, -30, -20, -15, -10, - 5, 0, 5, 10, 15, 20, 30, 45, 60, 75, 90};
		for (int i = 0; i < 19; ++i) {
			trackSensAngle[index][i] = a[i];
		}
	}
	else if (TRACK_SENSOR_AGR == 1) {
		float a[19] = {-45, -19, -12, -7, -4, -2.5, -1.7, -1, -.5, 0, .5, 1, 1.7, 2.5, 4, 7, 12, 19, 45};
		for (int i = 0; i < 19; ++i) {
			trackSensAngle[index][i] = a[i];
		}
	}
	else {
		for (int i = 0; i < 19; ++i) {
			trackSensAngle[index][i] = -90 + 10 * i;
		}
	}
		

	focusSens[index] = new Sensors(car, 5);//ML
	for (int i = 0; i < 5; ++i) {//ML
		focusSens[index]->setSensor(i, (car->_focusCmd) + i - 2, 200);//ML
	}//ML

		// Initialization of track sensors
	trackSens[index] = new Sensors(car, 19);
	for (int i = 0; i < 19; ++i) {
		trackSens[index]->setSensor(i, trackSensAngle[index][i], __SENSORS_RANGE__);
	}
	// Initialization of opponents sensors
	oppSens[index] = new ObstacleSensors(36, curTrack, car, sit, __SENSORS_RANGE__);

	// Zero data store
	memset((void*)tlData,0,sizeof(TORCSData_t));
	tlData->controlFlag = TL_NO_DATA;
	printf("matlabUoB %d: Online\n",index);
}

int lastEnable = 0;
/* Called once per robot update */
static int oncePerCycle(double t) {

	/* Inform user when enabling/disabling */
	if (lastEnable != tlData->enable) {
		if (tlData->enable) {
			printf("matlabUoB robots: Enabled!\n");
		}
		else {
			printf("matlabUoB robots: Disabled!\n");
		}
	}
	lastEnable = tlData->enable;

	/* If we are disabled, occasionally let the user know we're still alive, but otherwise do nothing */
	if (tlData->enable < 1) {
		tlData->controlFlag = TL_READY;	/* Tell Simulink we're ready and waiting... */
		if (fmod(t, 10) <= RCM_MAX_DT_SIMU) {
			printf("matlabUoB robots: Disabled (t=%.0fs)\n", t);
		}
		return -1;
	} else {
		tlData->controlFlag = TL_NO_DATA;
	}

	/* We want to block TORCS until we have an update from Simulink this ensures the two remain in sync
	Ensure we timeout after 10s so we don't block forever if Simulink dies on us */
	int startT = std::time(0);
	int timeout = 0;
	while (tlData->enable == 1 && tlData->controlFlag == TL_NO_DATA) {
		if ((std::time(0) - startT) > 5) {
			startT = std::time(0);
			switch (timeout) {
			case 0:
				printf("matlabUoB robots: No data from Simulink in the last 5 seconds... waiting 5 more...\n");
				timeout++;
				break;
			case 1:
				printf("matlabUoB robots: No data from Simulink in the last 10 seconds... disabling robot...\n");
				tlData->enable = 0;
				return -1;
			}
		}
	}
	return 0;
}

double lastTime = 0.0;
/* Drive during race */
static void drive(int index, tCarElt* car, tSituation *s) {
	/* We don't want control before the race starts */
	if (s->currentTime < 0) {
		return;
	}

#ifdef TL_ENABLE_RESTARTS
	/* If Simulink has requested a restart process this before anything else */
	if (tlData->controlFlag == TL_RESTART_RACE) {
		car->ctrl.askRestart = true;
		return;
	}
#endif

	if ((s->currentTime - lastTime) > RCM_MAX_DT_ROBOTS / 2) {
		if (oncePerCycle(s->currentTime) < 0) {
			return;
		} else {
			lastTime = s->currentTime;
		}
	}

	// computing distance to middle
	double dist_to_middle = 2 * car->_trkPos.toMiddle / (car->_trkPos.seg->width);
	// computing the car angle wrt the track axis
	double angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
	NORM_PI_PI(angle); // normalize the angle between -PI and + PI

	//Update focus sensors' angle
	for (int i = 0; i < 5; ++i) {
		focusSens[index]->setSensor(i, (car->_focusCmd) + i - 2, 200);
	}

	// update the value of track sensors only as long as the car is inside the track
	double trackSensorOut[19];
	double focusSensorOut[5];//ML
	if (dist_to_middle <= 1.0 && dist_to_middle >= -1.0)
	{
		trackSens[index]->sensors_update();
		for (int i = 0; i < 19; ++i)
		{
			trackSensorOut[i] = trackSens[index]->getSensorOut(i);
			//if (getNoisy())
				//trackSensorOut[i] *= normRand(1, __NOISE_STD__);
		}
		focusSens[index]->sensors_update();//ML
		if ((car->_focusCD <= car->_curLapTime + car->_curTime)//ML Only send focus sensor reading if cooldown is over
			&& (car->_focusCmd != 360))//ML Only send focus reading if requested by client
		{//ML
			for (int i = 0; i < 5; ++i)
			{
				focusSensorOut[i] = focusSens[index]->getSensorOut(i);
				//if (getNoisy())
					//focusSensorOut[i] *= normRand(1, __FOCUS_NOISE_STD__);
			}
			car->_focusCD = car->_curLapTime + car->_curTime + 1.0;//ML Add cooldown [seconds]
		}//ML
		else//ML
		{//ML
			for (int i = 0; i < 5; ++i)//ML
				focusSensorOut[i] = -1;//ML During cooldown send invalid focus reading
		}//ML
	}
	else
	{
		for (int i = 0; i < 19; ++i)
		{
			trackSensorOut[i] = -1;
		}
		for (int i = 0; i < 5; ++i)
		{
			focusSensorOut[i] = -1;
		}
	}

	// update the value of opponent sensors
	//double oppSensorOut[36];
	//oppSens[index]->sensors_update(s);
	//for (int i = 0; i < 36; ++i)
	//{
	//	oppSensorOut[i] = oppSens[index]->getObstacleSensorOut(i);
	//	//if (getNoisy())
	//		//oppSensorOut[i] *= normRand(1, __OPP_NOISE_STD__);
	//}

	double wheelSpinVel[4];
	for (int i = 0; i<4; ++i)
	{
		wheelSpinVel[i] = car->_wheelSpinVel(i);
	}

	//if (prevDist[index]<0)
	//{
	//	prevDist[index] = car->race.distFromStartLine;
	//}
	//double curDistRaced = car->race.distFromStartLine - prevDist[index];
	//prevDist[index] = car->race.distFromStartLine;
	//if (curDistRaced>100)
	//{
		//curDistRaced -= curTrack->length;
	//}
	//if (curDistRaced<-100)
	//{
		//curDistRaced += curTrack->length;
	//}

	//distRaced[index] += curDistRaced;

	double totdist = curTrack->length * (car->race.laps - 1) + car->race.distFromStartLine;
	
	// TODO try this?
	//("opponents", oppSensorOut, 36);

	/* If we've got this far then we have new data from Simulink, update our control */
	car->ctrl.accelCmd = tlData->vehicle[index].control.throttle;
	car->ctrl.brakeCmd = tlData->vehicle[index].control.brake;
	car->ctrl.clutchCmd = tlData->vehicle[index].control.clutch;
	car->ctrl.gear = tlData->vehicle[index].control.gear;
	car->ctrl.steer = tlData->vehicle[index].control.steering;

	/* Update the outputs of our vehicle */
	tlData->vehicle[index].data.angleP = angle;
	tlData->vehicle[index].data.curLapTime = double(car->_curLapTime);
	tlData->vehicle[index].data.distFromStart = double(car->race.distFromStartLine);
	tlData->vehicle[index].data.totalDistFromStart = totdist;
	tlData->vehicle[index].data.fuel = double(car->_fuel);
	tlData->vehicle[index].data.gear = double(car->_gear);
	tlData->vehicle[index].data.lastLapTime = double(car->_lastLapTime);
	tlData->vehicle[index].data.racePos = double(car->race.pos);
	tlData->vehicle[index].data.track[0] = trackSensorOut[0];
	tlData->vehicle[index].data.track[1] = trackSensorOut[1];
	tlData->vehicle[index].data.track[2] = trackSensorOut[2];
	tlData->vehicle[index].data.track[3] = trackSensorOut[3];
	tlData->vehicle[index].data.track[4] = trackSensorOut[4];
	tlData->vehicle[index].data.track[5] = trackSensorOut[5];
	tlData->vehicle[index].data.track[6] = trackSensorOut[6];
	tlData->vehicle[index].data.track[7] = trackSensorOut[7];
	tlData->vehicle[index].data.track[8] = trackSensorOut[8];
	tlData->vehicle[index].data.track[9] = trackSensorOut[9];
	tlData->vehicle[index].data.track[10] = trackSensorOut[10];
	tlData->vehicle[index].data.track[11] = trackSensorOut[11];
	tlData->vehicle[index].data.track[12] = trackSensorOut[12];
	tlData->vehicle[index].data.track[13] = trackSensorOut[13];
	tlData->vehicle[index].data.track[14] = trackSensorOut[14];
	tlData->vehicle[index].data.track[15] = trackSensorOut[15];
	tlData->vehicle[index].data.track[16] = trackSensorOut[16];
	tlData->vehicle[index].data.track[17] = trackSensorOut[17];
	tlData->vehicle[index].data.track[18] = trackSensorOut[18];
	tlData->vehicle[index].data.trackPos = dist_to_middle;
	tlData->vehicle[index].data.wheelSpinVel[0] = wheelSpinVel[0];
	tlData->vehicle[index].data.wheelSpinVel[1] = wheelSpinVel[1];
	tlData->vehicle[index].data.wheelSpinVel[2] = wheelSpinVel[2];
	tlData->vehicle[index].data.wheelSpinVel[3] = wheelSpinVel[3];
	tlData->vehicle[index].data.z = car->_pos_Z - RtTrackHeightL(&(car->_trkPos));
	tlData->vehicle[index].data.focus[0] = focusSensorOut[0];
	tlData->vehicle[index].data.focus[1] = focusSensorOut[1];
	tlData->vehicle[index].data.focus[2] = focusSensorOut[2];
	tlData->vehicle[index].data.focus[3] = focusSensorOut[3];
	tlData->vehicle[index].data.focus[4] = focusSensorOut[4];

#ifdef TL_USE_DOUBLE_POSITION
	tlData->vehicle[index].data.position[0] = car->pub.DynGCg.posD.x;
	tlData->vehicle[index].data.position[1] = car->pub.DynGCg.posD.y;
	tlData->vehicle[index].data.position[2] = car->pub.DynGCg.posD.z;
#else
	tlData->vehicle[index].data.position[0] = car->pub.DynGCg.pos.x;
	tlData->vehicle[index].data.position[1] = car->pub.DynGCg.pos.y;
	tlData->vehicle[index].data.position[2] = car->pub.DynGCg.pos.z;
#endif
	tlData->vehicle[index].data.velocity[0] = car->pub.DynGCg.vel.x;
	tlData->vehicle[index].data.velocity[1] = car->pub.DynGCg.vel.y;
	tlData->vehicle[index].data.velocity[2] = car->pub.DynGCg.vel.z;
	tlData->vehicle[index].data.acceleration[0] = car->pub.DynGCg.acc.x;
	tlData->vehicle[index].data.acceleration[1] = car->pub.DynGCg.acc.y;
	tlData->vehicle[index].data.acceleration[2] = car->pub.DynGCg.acc.z;
	tlData->vehicle[index].data.angle[0] = car->pub.DynGCg.pos.ax;
	tlData->vehicle[index].data.angle[1] = car->pub.DynGCg.pos.ay;
	tlData->vehicle[index].data.angle[2] = car->pub.DynGCg.pos.az;
	tlData->vehicle[index].data.angularVelocity[0] = car->pub.DynGC.vel.ax;
	tlData->vehicle[index].data.angularVelocity[1] = car->pub.DynGC.vel.ay;
	tlData->vehicle[index].data.angularVelocity[2] = car->pub.DynGC.vel.az;
	
	double error = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
	error = fmod(error + 2*PI,2*PI);	// Normalise heading angle
	if (error > PI) {
		error = error - 2*PI;
	}
	tlData->vehicle[index].data.headingError = error;
	tlData->vehicle[index].data.lateralError = car->_trkPos.toMiddle;
	tlData->vehicle[index].data.roadDistance = RtGetDistFromStart(car);
	double curvature = 0.0;
	if (car->_trkPos.seg->radius > 0) {
		curvature = 1.0/car->_trkPos.seg->radius;
		if (car->_trkPos.seg->type == TR_RGT) {
			curvature *= -1;
		}
	}
	tlData->vehicle[index].data.roadCurvature = curvature;
	tlData->vehicle[index].data.engineRPM = car->_enginerpm * 9.54929659; // For some reason RPM is actually rad/s!

}


// End of the current race.
static void endRace(int index, tCarElt *car, tSituation *s) {
	if (trackSens != NULL)
	{
		delete trackSens[index];
		trackSens[index] = NULL;
	}

	if (oppSens[index] != NULL)
	{
		delete oppSens[index];
		oppSens[index] = NULL;
	}
	if (focusSens[index] != NULL)//ML
	{
		delete focusSens[index];
		focusSens[index] = NULL;
	}

	printf("matlabUoB %d: Offline\n", index);
}


// Called before the module is unloaded.
static void shutdown(int index) {
	if (trackSens[index] != NULL)
	{
		delete trackSens[index];
		trackSens[index] = NULL;
	}
	if (focusSens[index] != NULL)//ML
	{
		delete focusSens[index];
		focusSens[index] = NULL;
	}
	if (oppSens[index] != NULL)
	{
		delete oppSens[index];
		oppSens[index] = NULL;
	}
	tlCloseSharedMemory();
}
