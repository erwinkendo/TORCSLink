/***************************************************************************
 
    file                 : sensors.cpp
    copyright            : (C) 2007 Alessandro Prete, Daniele Loiacono
 
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *  
 ***************************************************************************/                                                 
 
#include "sensors.h"

#define HALF_PI PI/2

Sensors::Sensors(tCarElt *car, int sensors_number)
{
	sensors_num = sensors_number;
	sensor = new SingleSensor[sensors_num];
	for (int i = 0; i < sensors_num; i++) sensor[i].init(car);
}


Sensors::~Sensors()
{
	delete [] sensor;
}


void Sensors::sensors_update()
{
	for (int i = 0; i < sensors_num; i++) sensor[i].update();
}


void Sensors::setSensor(int sensor_id, double angle, double range)
{ 
	sensor[sensor_id].setSingleSensor(angle, range); 
}


double Sensors::getSensorOut(int sensor_id)
{ 
	return sensor[sensor_id].getSingleSensorOut(); 
}


void SingleSensor::setSingleSensor(double angle, double range)
{
	sensor_angle = (angle*PI)/180;
	sensor_range = range;
}


void SingleSensor::update()
{
	//Angolo del sensore relativo al segmento del tracciato
	double relative_sensor_angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw + sensor_angle; 

/*	printf ("\nabsolute_car_angle:               %f (degree)", (car->_yaw*180)/PI);
	printf ("\nabsolute_sensor_angle:            %f (degree)", (absolute_sensor_angle*180)/PI);
	printf ("\nabsolute_track_angle:             %f (degree)", (absolute_track_angle*180)/PI);
	printf ("\nrelative_sensor_angle:            %f (degree)", (relative_sensor_angle*180)/PI);
	printf ("\nsensor_angle:					 %f (degree)", (sensor_angle*180)/PI);*/
	
	NORM_PI_PI(relative_sensor_angle);
	
/*	printf ("\nrelative_sensor_angle normalized: %f (degree)", (relative_sensor_angle*180)/PI);
	printf ("\nX1:                               %f (meters)", car->_trkPos.toLeft);
	printf ("\nX2:                               %f (meters)", car->_trkPos.toRight);
	printf ("\nY:                                %f (meters or arc)", car->_trkPos.toStart);
	printf ("\nTo Middle:                        %f (meters)", car->_trkPos.toMiddle);
	printf ("\nTrack Segment Start Width:        %f (meters)", car->_trkPos.seg->startWidth);
	printf ("\nTrack Segment End Width:          %f (meters)", car->_trkPos.seg->endWidth);
	printf ("\nTrack Segment Length:             %f (meters)\n", car->_trkPos.seg->length);*/

	switch (car->_trkPos.seg->type) 
	{
		case 3: sensor_out = sensor_calc_str(car->_trkPos.seg, car->_trkPos.toLeft, car->_trkPos.toStart, -relative_sensor_angle, sensor_range);
				break;
		case 2:	sensor_out = sensor_calc_lft_rgt(car->_trkPos.seg, car->_trkPos.toLeft, car->_trkPos.toStart, -relative_sensor_angle, sensor_range);
				break;
		case 1:	sensor_out = sensor_calc_lft_rgt(car->_trkPos.seg, car->_trkPos.toLeft, car->_trkPos.toStart, -relative_sensor_angle, sensor_range);
				break;
	}
}


double SingleSensor::sensor_calc_str(tTrackSeg *seg, double x_coord, double y_coord, double angle, double remaining_range) 
{
	double x_sx = x_coord;						//meters
	double x_dx = (seg->startWidth) - x_coord;	//meters
	double y_up = (seg->length) - y_coord;		//meters
	double y_dn = y_coord;						//meters

	if (cos(angle) == 0)
	{
		if (angle > 0)
		{
			if (x_sx < remaining_range) return x_sx;
			else return remaining_range;
		}
		else 
		{
			if (x_dx < remaining_range) return x_dx;
			else return remaining_range;
		}
	}
	else
		if (cos(angle) > 0 && sin(angle) >= 0)
		{
			if ((x_sx/y_up) > tan(angle)) //potrei incontrare il bordo superiore, controllare il range 
			{
				double partial = y_up/cos(angle);
				if (partial >= remaining_range) return remaining_range;
				else
				{
					double x = x_sx - (partial * sin(angle));
					double partial_returned=0.1;
					switch (seg->next->type) 
					{
						case 3:	partial_returned = sensor_calc_str(seg->next, x, 0, angle, (remaining_range - partial));
								break;
						case 2:	partial_returned = sensor_calc_lft_rgt(seg->next, x, 0, angle, (remaining_range - partial));
								break;
						case 1:	partial_returned = sensor_calc_lft_rgt(seg->next, x, 0, angle, (remaining_range - partial));
								break;
					}
					return (partial_returned + partial);
				}
			}
			else //potrei incontrare il bordo sinistro, controllare il range
			{
				double temp_val = x_sx / cos((HALF_PI) - angle);
				if (temp_val >= remaining_range) return remaining_range;
				else return temp_val;
			}
		}
	else
		if (cos(angle) > 0 && sin(angle) < 0)
		{
			if ((x_dx/y_up) > -tan(angle)) //potrei incontrare il bordo superiore, controllare il range
			{
				double partial = y_up/cos(angle);
				if (partial >= remaining_range) return remaining_range;
				else
				{
					double x = x_sx + (partial * sin(-angle));
					double partial_returned=0.1;
					switch (seg->next->type)
					{
						case 3:	partial_returned = sensor_calc_str(seg->next, x, 0, angle, (remaining_range - partial));
								break;
						case 2:	partial_returned = sensor_calc_lft_rgt(seg->next, x, 0, angle, (remaining_range - partial));
								break;
						case 1:	partial_returned = sensor_calc_lft_rgt(seg->next, x, 0, angle, (remaining_range - partial));
								break;
					}
					return (partial_returned + partial);
				}
			}
			else //potrei incontrare il bordo destro, controllare il range
			{
				double temp_val = x_dx / cos((HALF_PI) + angle);
				if (temp_val >= remaining_range) return remaining_range;
				else return temp_val;
			}
		}
	else
		if (cos(angle) < 0 && sin(angle) >= 0)
		{
			if ((x_sx/y_dn) > tan(PI-angle)) //potrei incontrare il bordo inferiore, controllare il range
			{
				double partial = y_dn/-cos(angle);
				if (partial >= remaining_range) return remaining_range;
				else
				{
					double x = x_sx - (partial * sin(PI-angle));
					double partial_returned=0.1;
					switch (seg->prev->type) 
					{
						case 3:	partial_returned = sensor_calc_str(seg->prev, x, seg->prev->length, angle, (remaining_range - partial));
								break;
						case 2:	partial_returned = sensor_calc_lft_rgt(seg->prev, x, seg->prev->arc, angle, (remaining_range - partial));
								break;
						case 1:	partial_returned = sensor_calc_lft_rgt(seg->prev, x, seg->prev->arc, angle, (remaining_range - partial));
								break;
					}
					return (partial_returned + partial);
				}
			}
			else //potrei incontrare il bordo sinistro, controllare il range
			{
				double temp_val = x_sx / cos(angle - (HALF_PI));
				if (temp_val >= remaining_range) return remaining_range;
				else return temp_val;
			}
		}
	else
		if (cos(angle) < 0 && sin(angle) < 0)
		{
			if ((x_dx/y_dn) > tan(PI+angle)) //potrei incontrare il bordo inferiore, controllare il range
			{
				double partial = y_dn / cos(PI + angle);
				if (partial >= remaining_range) return remaining_range;
				else
				{
					double x = x_sx + partial * sin(PI + angle);
					double partial_returned=0.1;
					switch (seg->prev->type) 
					{
						case 3:	partial_returned = sensor_calc_str(seg->prev, x, seg->prev->length, angle, (remaining_range - partial));
								break;
						case 2:	partial_returned = sensor_calc_lft_rgt(seg->prev, x, seg->prev->arc, angle, (remaining_range - partial));
								break;
						case 1:	partial_returned = sensor_calc_lft_rgt(seg->prev, x, seg->prev->arc, angle, (remaining_range - partial));
								break;
					}
					return (partial_returned + partial);
				}
			}
			else //potrei incontrare il bordo destro, controllare il range
			{
				double temp_val = x_dx / cos(-angle - (HALF_PI));
				if (temp_val >= remaining_range) return remaining_range;
				else return temp_val;
			}
		}
	else return remaining_range;
}


double SingleSensor::sensor_calc_lft_rgt(tTrackSeg *seg, double x_coord, double y_coord, double angle, double remaining_range) 
{
	double x_sx = x_coord;						//meters
	double x_dx = (seg->startWidth) - x_sx;		//meters
	double y_up = (seg->arc) - y_coord;			//radians
	double y_dn = y_coord;						//radians
	double radius_max, radius_min, radius_x, temp_angle;

	if (seg->type == 2)
	{
		radius_max = seg->radiusr;
		radius_min = seg->radiusl;
		radius_x = radius_min + x_sx;
		temp_angle = angle;
	}
	else
	{
		radius_max = seg->radiusl;
		radius_min = seg->radiusr;
		radius_x = radius_min + x_dx;
		temp_angle = -angle;
	}
	
	double dist_returned;
	double angle_returned;
	double x_returned;
	if (check_max_circle_intersect(temp_angle, radius_max, radius_x, y_dn, y_up, remaining_range, dist_returned)) return dist_returned;
	else if	(check_min_circle_intersect(temp_angle, radius_min, radius_x, y_dn, y_up, remaining_range, dist_returned)) return dist_returned;
	else if (check_up_border_intersect(temp_angle, radius_min, radius_x, y_up, remaining_range, dist_returned, angle_returned, x_returned))
	{
		double partial_returned=0.1;
//		printf ("\nRecursive Call for next segment\n");
		
		// Correction of outputs if we are currently in a right turn (Daniele Loiacono - 3/2009)
		if (seg->type == 1)
		{
			x_returned = seg->startWidth - x_returned;
			angle_returned = -angle_returned;
		}

		if (seg->next->type==3) 
		{
			partial_returned = sensor_calc_str(seg->next, x_returned, 0, angle_returned, (remaining_range - dist_returned));
		} 
		else
		{
			partial_returned = sensor_calc_lft_rgt(seg->next, x_returned, 0, angle_returned, (remaining_range - dist_returned));

		}
		return (partial_returned + dist_returned);
	}
	else if (check_down_border_intersect(temp_angle, radius_min, radius_x, y_dn, remaining_range, dist_returned, angle_returned, x_returned))
	{
		double partial_returned=0.1;
//		printf ("\nRecursive Call for prev segment\n");

		// Correction of outputs if we are currently in a right turn (Daniele Loiacono - 3/2009)
		if (seg->type == 1)
		{
			x_returned = seg->startWidth - x_returned;
			angle_returned = -angle_returned;
		}

		if (seg->prev->type==3) 
		{
			partial_returned = sensor_calc_str(seg->prev, x_returned, seg->prev->length, angle_returned, (remaining_range - dist_returned));
					
		}
		else
		{
			partial_returned = sensor_calc_lft_rgt(seg->prev, x_returned, seg->prev->arc, angle_returned, (remaining_range - dist_returned));
		}		
		return (partial_returned + dist_returned);
	}
	else return remaining_range;
}


bool SingleSensor::check_max_circle_intersect(double angle, double radius_max, double radius_x, double y_dn, double y_up, double remaining_range, double &dist_returned)
{
	if (angle < 0)
	{
		if (angle >= -(HALF_PI))
		{
			double check = radius_x * cos(-angle);
			double beta = acos(check/radius_max);
			double gamma = beta + angle;
			if (gamma <= y_up) //il sensore potrebbe incontrare il cerchio max in questo segmento
			{
				double check_dist = radius_x * sin(-angle);
				double z = radius_max * sin(beta);
				double dist = z - check_dist;
				if (dist <= remaining_range)
				{ 
					dist_returned = dist;
//					printf ("\nMax Circle Intersection (-90 < angle < 0)\n");
					return true;
				}
				else return false;
			}
			else return false;
		}
		else // if (angle < -(HALF_PI))
		{
			double check = radius_x * cos(PI + angle);
			double beta = acos(check/radius_max);
			double gamma = beta - angle - PI;
			if (gamma <= y_dn) //il sensore potrebbe incontrare il cerchio max in questo segmento
			{
				double check_dist = radius_x * sin(PI + angle);
				double z = radius_max * sin(beta);
				double dist = z - check_dist;
				if (dist <= remaining_range)
				{ 
					dist_returned = dist;
//					printf ("\nMax Circle Intersection (-180 < angle < -90)\n");
					return true;
				}
				else return false;
			}
			else return false;
		}
	}
	
	else if (angle > 0)
	{
		if (angle <= (HALF_PI))
		{
			double check = radius_x * cos(angle);
			double beta = acos(check/radius_max);
			double gamma = beta + angle;
			if (gamma <= y_up) //il sensore potrebbe incontrare il cerchio max in questo segmento
			{
				double check_dist = radius_x * sin(angle);
				double z = radius_max * sin(beta);
				double dist = z + check_dist;
				if (dist <= remaining_range)
				{ 
					dist_returned = dist;
//					printf ("\nMax Circle Intersection (0 < angle < 90)\n");
					return true;
				}
				else return false;
			}
			else return false;
		}
		else // if (angle > (HALF_PI))
		{
			double check = radius_x * cos(PI + angle);
			double beta = acos(check/radius_max);
			double gamma = beta + angle + PI;
			if (gamma <= y_up) //il sensore potrebbe incontrare il cerchio max in questo segmento
			{
				double check_dist = radius_x * sin(PI + angle);
				double z = radius_max * sin(beta);
				double dist = z + check_dist;
				if (dist <= remaining_range)
				{ 
					dist_returned = dist;
//					printf ("\nMax Circle Intersection (90 < angle < 180)\n");
					return true;
				}
				else return false;
			}
			else return false;
		}
	}
	else return false;
}


bool SingleSensor::check_min_circle_intersect(double angle, double radius_min, double radius_x, double y_dn, double y_up, double remaining_range, double &dist_returned)
{
	if (angle > 0 && angle <= (HALF_PI))
	{
		double check = radius_x * cos(angle);
		double beta = acos(check/radius_min);
		double gamma = angle - beta;
		if (gamma <= y_up) //il sensore potrebbe incontrare il cerchio min in questo segmento
		{
			double check_dist = radius_x * sin(angle);
			double z = radius_min * sin(beta);
			double dist = check_dist - z;
			if (dist <= remaining_range)
			{ 
				dist_returned = dist;
//				printf ("\nMin Circle Intersection (0 < angle < 90)\n");
				return true;
			}
			else return false;
		}
		else return false;
	}
	else if (angle > 0 && angle > (HALF_PI))
	{
		double check = radius_x * cos(PI - angle);
		double beta = acos(check/radius_min);
		double gamma = PI - beta - angle;
		if (gamma <= y_dn && gamma >= 0.0) //il sensore potrebbe incontrare il cerchio min in questo segmento
		{
			double check_dist = radius_x * sin(PI - angle);
			double z = radius_min * sin(beta);
			double dist = check_dist - z;
			if (dist <= remaining_range)
			{ 
				dist_returned = dist;
//				printf ("\nMin Circle Intersection (90 < angle < 180)\n");
				return true;
			}
			else return false;
		}
		else return false;
	}
	else return false;
}


bool SingleSensor::check_up_border_intersect(double angle, double radius_min, double radius_x, double y_up, double remaining_range, double &dist_returned, double &angle_returned, double &x_returned)
{
	if (angle < (HALF_PI) && angle > -(HALF_PI))
	{
		if (angle >= y_up)
		{
			double check_dist = radius_x * sin(y_up);
			double dist = check_dist / cos(angle - y_up);
			if (dist < remaining_range) //il sensore incontra il bordo superiore di questo segmento
			{
				double z = dist * sin(angle - y_up);
				double check = radius_x * cos(y_up);
				dist_returned = dist;
				angle_returned = angle - y_up;
				x_returned =  check - z - radius_min;
//				printf ("\nUp Boarder Intersection\n");
				return true;
			}
			else return false;
		}
		else
		{
			double check_dist = radius_x * sin(y_up);
			double dist = check_dist / cos(y_up - angle);
			if (dist < remaining_range) //il sensore incontra il bordo superiore di questo segmento
			{
				double z = dist * sin(y_up - angle);
				double check = radius_x * cos(y_up);
				dist_returned = dist;
				angle_returned = - (y_up - angle);
				x_returned =  check + z - radius_min;
//				printf ("\nUp Boarder Intersection\n");
				return true;
			}
			else return false;
		}
	}
	else return false;
}


bool SingleSensor::check_down_border_intersect(double angle, double radius_min, double radius_x, double y_dn, double remaining_range, double &dist_returned, double &angle_returned, double &x_returned)
{
	if (angle > (HALF_PI) || angle < -(HALF_PI))
	{
		if (angle > 0 && angle <= (PI - y_dn))
		{
			double check_dist = radius_x * sin(y_dn);
			double dist = check_dist / cos(PI - angle - y_dn);
			if (dist < remaining_range) //il sensore incontra il bordo inferiore di questo segmento
			{
				double z = dist * sin(PI - angle - y_dn);
				double check = radius_x * cos(y_dn);
				dist_returned = dist;
				angle_returned = angle + y_dn;
				x_returned =  check - z - radius_min;
//				printf ("\nDown Boarder Intersection\n");
				return true;
			}
			else return false;
		}
		else if (angle > 0 && angle > (PI - y_dn))
		{
			double check_dist = radius_x * sin(y_dn);
			double dist = check_dist / cos(y_dn + angle - PI);
			if (dist < remaining_range) //il sensore incontra il bordo inferiore di questo segmento
			{
				double z = dist * sin(y_dn + angle - PI);
				double check = radius_x * cos(y_dn);
				dist_returned = dist;
				angle_returned = -((2 * PI) - angle - y_dn);
				x_returned =  check + z - radius_min;
//				printf ("\nDown Boarder Intersection\n");
				return true;
			}
			else return false;
		}
		else
		{
			double check_dist = radius_x * sin(y_dn);
			double dist = check_dist / cos(y_dn + angle + PI);
			if (dist < remaining_range) //il sensore incontra il bordo inferiore di questo segmento
			{
				double z = dist * sin(y_dn + angle + PI);
				double check = radius_x * cos(y_dn);
				dist_returned = dist;
				angle_returned = -(-angle - y_dn);
				x_returned =  check + z - radius_min;
//				printf ("\nDown Boarder Intersection\n");
				return true;
			}
			else return false;
		}
	}
	else return false;
}
