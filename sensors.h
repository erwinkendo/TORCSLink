/***************************************************************************
 
    file                 : sensors.h
    copyright            : (C) 2007 Alessandro Prete
 
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *  
 ***************************************************************************/ 
#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <stdio.h>
#include <math.h>
#include <car.h>
#include <robottools.h>
#include <raceman.h>

class SingleSensor {
	public:
		inline void init(CarElt *car) { this->car = car; }

		void setSingleSensor(double angle, double range);
		inline double getSingleSensorOut() { return sensor_out; }
		void update();

	protected:
		double sensor_calc_str(tTrackSeg *seg, double x, double y, double sensor_angle, double remaining_range);
		double sensor_calc_lft_rgt(tTrackSeg *seg, double x, double y, double sensor_angle, double remaining_range);
		
		bool check_max_circle_intersect(double angle, double radius_max, double radius_x, double y_dn, double y_up, double remaining_range, double &dist_returned);
		bool check_min_circle_intersect(double angle, double radius_min, double radius_x, double y_dn, double y_up, double remaining_range, double &dist_returned);
		bool check_up_border_intersect(double angle, double radius_min, double radius_x, double y_up, double remaining_range, double &dist_returned, double &angle_returned, double &x_returned);
		bool check_down_border_intersect(double angle, double radius_min, double radius_x, double y_dn, double remaining_range, double &dist_returned, double &angle_returned, double &x_returned);

		double sensor_range; //meters
		double sensor_angle; //radians
		double sensor_out;	//meters

		tCarElt *car;
};

class Sensors {
	public:
		Sensors(tCarElt *car, int sensors_number);
		~Sensors();

		//Params:
		//			id from 0 to sensors_number-1
		//			angle in degree, positive for left angle, negative for right angle
		//			range in meters
		void setSensor(int sensor_id, double angle, double range);

		//Return Value:
		//				sensor range if no boarder detected in the range
		//				border distance if boarder detected
		//note:	if the car reach the border the value is 0.
		//		if the car go behind the track border the value in < 0, don't consider it!
		double getSensorOut(int sensor_id);

		void sensors_update();

	protected:
		SingleSensor *sensor;
		int sensors_num;
};

#endif
