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

#ifndef _CARDATA_H_
#define _CARDATA_H_

#include "variables.h"

using namespace std;

class carData{
  
  public:  

    static const double CAR_MAX_DEFAULT_SPEED; /* default max speed, if not defined in the XML */
    
    carData(tCarElt *mycar, tTrack *track);

    void updateCar(tSituation *s);

    /** getter */
    
    inline double getCW() 			{ return CW; }
    inline double getCA()			{ return CA; }
    inline double getSpeedSqr()		{ return speedSqr; }
    inline double getSpeedOpp()		{ return speedOpp; }
    inline tCarElt * getCarPnt()		{ return car; }
    inline int    getCarIndex()		{ return car->_driverIndex; }
    inline double getMass() 			{ return mass; }
    inline double getDrivenWheelSpeed()	{ return (this->*GET_DRIVEN_WHEEL_SPEED)();}
    inline double getAccel(double speed)	{ return (this->*GET_ACCEL_FUNCT)(speed);}
    inline double getCurrentFriction() 	{ return car->_trkPos.seg->surface->kFriction;}
    inline double getMaxSpeed()   		{ return maxSpeed; }
    inline double getMaxAccel()		{ return maxAccel; }
    inline double getMaxDecel()		{ return maxDecel; }
    inline double getMinTurn()			{ return minTurn;  }
    inline double getFriction()   		{ return friction; } 
    inline int    getMode()  	   		{ return mode;     }
    inline double getStuckTime()		{ return stuckTime; }
    inline double getGearUp()			{ return car->_gearRatio[car->_gear + car->_gearOffset]; }
    inline double getGearDown()		{ return car->_gearRatio[car->_gear + car->_gearOffset - 1]; }
    inline const char * getBasePath()		{ return basePath; }
    inline bool   startMode()			{ return mode == START; }
    inline bool   recoveryMode()		{ return mode == RECOVERY; }
    inline bool   normalMode()			{ return mode == NORMAL; }
    inline bool   stuckMode()			{ return mode == STUCK; }
  
    //TODO behaviour, (matrix).
    
    /** setter */
  
    inline void   setMode(int x)  		{ mode = x; }
    inline void   setBehaviour(int x)		{ behaviour = x; }
    
  private:
        
    int drivetrain, mode, behaviour; /* variables --> enum mode */        
    double lastTime , lastSpeed , CA , CW , CARMASS , speedSqr , mass , maxSpeed , friction;
    double stuckTime, minTurn, maxAccel, maxDecel , speedOpp;
    char  basePath[255];
    tCarElt *car; 
         
    /** init methods */
    
    double initCA();
    double initCW();
    void  initTrainType();
    
    /* during the initTrainType these two pointers point to the correct functions */
    
    double (carData::*GET_DRIVEN_WHEEL_SPEED)();
    double (carData::*GET_ACCEL_FUNCT)(double speed);
    
    /** used during execution */
    
    double filterTCL_RWD();
    double filterTCL_FWD();
    double filterTCL_4WD();
    double getAccel_RWD(double speed);
    double getAccel_FWD(double speed);
    double getAccel_4WD(double speed);

};


    
#endif

