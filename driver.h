/***************************************************************************

    file                 : Driver.h
    created              : Sat Mar 2 17:02:55 CET 2013
    copyright            : (C) 2002 Daniel Frassinelli

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

#include "carData.h"
#include "trajectoryPlanner.h"
#include "opponent.h"
#include "logger.h"

class Driver{
  public:
    Driver(int index);
    ~Driver();
    
    void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
    void newrace(int index, tCarElt* car, tSituation *s); 
    void drive(int index, tSituation *s); 
    void endrace(int index, tSituation *s);
    int  pitCommand(tCarElt* car, tSituation *s);

  private:
    
    /** check variables */
    
    static const double STUCK_MAX_ANGLE;
    static const double STUCK_MAX_DIST;
    static const double STUCK_MAX_SPEED;
    static const int    STUCK_MAX_COUNT;
    
    /** Controller varaibles */
    
    static const double ACCEL_PI_KI;		// proportinal constant
    static const double ACCEL_PI_KE;  		// integrative constant
    static const double ACCEL_INTEG_LIMIT;	// we have to limit the error because sometimes can be infinite
    static const double ACCEL_CONST_LIMIT;	// we have to limit the intergral because grows too much
    
    static const double STEER_K_SOFT;		//carlike model and controller Ksoft
    static const double STEER_K_SPEED;		//carlike model and controller Kspeed
    static const double STEER_K_YAW;		//carlike model and controller Kyaw
    
    static const double GEAR_UP_SPEED_GAIN;
    static const double GEAR_DOWN_SPEED_GAIN;
    static const double GEAR_MAX_TIME;
    
    static const double CLUTCH_FULL_MAX_TIME;
    static const double CLUTCH_SPEED;
    
    /** filter variables */
    
    static const double FILTER_ACCEL_START;			//Accel pedal at 0m/s
    static const double FILTER_ACCEL_START_MODE_SPEED;		//speed needed for start mode
    static const double RECOVERY_YAW_RATE;			//reduce the accel pedal when out of the track based on the yaw rate
    static const double RECOVERY_ACCEL_REDUCE;			//reduce the accel pedal when out of the track
    
    static const double ABS_MIN_SPEED;
    static const double ABS_SLIP;
    
    static const double RECOVERY_MAX_SPEED;			//Max speed when out of the track
    
    static const double TCL_SLIP;
    static const double TCL_RANGE; 

    int stuck, index;
    double mass , time_difference , stuckangle , allowedSpeed , trajangle , trajdist;
    double clutchTime , gearTime;

    tTrack    *track;
    tCarElt   *car;   		
    carData   *myCar;  
    opponents *allCars;
    logger    *log;
    vector <opponent *> * enemyCars; 
    trajectoryPlanner *trajectory;
    
    //data of the previous invocation of drive()
    
    typedef struct {
      double speed_x;
      double trajangle;
      double carYaw;
      double allowedSpeed;
      double accelError;
      double accelErrorSum; 
    } past_data;
    
    past_data last;
    
    /** checker */
      
    bool  isStuck();		//check if the car is stuck
    bool  isRecovery();		//check if the car is out of the track
    
    /** controller */

    double getAccel();		//obtain a value of accel [-1.0 ... 1.0]
    double getSteer();         //obtain a value of steer [-1.0 ... 1.0]
    int    getGear();  		//gear changing [1 - 6]
    double getClutch();		//obtain clutch (experimental)
    
    /** opponent filters */
    
    double filterAccelOpp(double accel);
    double filterSteerOpp(double steer);
    
    /** performance filters */
      
    double filterAccel(double accel);
    double filterSteer(double steer);
    double filterSpeed(double speed);
    double filterBrake(double brake);
    
    /** update methods */

    void  initialUpdate(tSituation *s); 
    void  finalUpdate(tSituation *s);

};

#endif

