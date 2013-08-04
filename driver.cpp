/***************************************************************************

    file                 : driver.cpp
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

#include "driver.h"

#include <iostream>

using namespace std;

/** check */

const double Driver::STUCK_MAX_ANGLE = (15*PI)/180;
const double Driver::STUCK_MAX_SPEED = 5.0;
const int    Driver::STUCK_MAX_COUNT = 100;
const double Driver::STUCK_MAX_DIST  = 3.0;

/** controller */

const double Driver::GEAR_UP_SPEED_GAIN = 0.9;
const double Driver::GEAR_DOWN_SPEED_GAIN = 0.4;
const double Driver::GEAR_MAX_TIME = 1.0; 

const double Driver::ACCEL_PI_KI = 0.0005;
const double Driver::ACCEL_PI_KE = 0.9;
const double Driver::ACCEL_INTEG_LIMIT = 100000.0;
const double Driver::ACCEL_CONST_LIMIT = 1000.0;

const double Driver::STEER_K_SOFT  = 50.0; 
const double Driver::STEER_K_SPEED  = 10.0;
const double Driver::STEER_K_YAW    = 0.1;

const double Driver::CLUTCH_SPEED = 5.0;
const double Driver::CLUTCH_FULL_MAX_TIME = 2.0;

/** filter */

const double Driver::FILTER_ACCEL_START = 0.65;
const double Driver::FILTER_ACCEL_START_MODE_SPEED = 2.0;

const double Driver::RECOVERY_YAW_RATE = 8.0;
const double Driver::RECOVERY_ACCEL_REDUCE = 0.065;

const double Driver::RECOVERY_MAX_SPEED = 20.0;

const double Driver::TCL_SLIP = 2.0;        
const double Driver::TCL_RANGE = 5.0;  

const double Driver::ABS_MIN_SPEED = 5.0;
const double Driver::ABS_SLIP      = 0.9;

Driver::Driver(int index){
  this->index = index;
  myCar = NULL;
  trajectory = NULL;
  allCars = NULL; 
  log = NULL;
  last = past_data();
}

Driver::~Driver(){
  if(myCar != NULL)
    delete myCar;
  if(trajectory != NULL)
    delete trajectory;
  if(allCars != NULL)
    delete allCars;
  if(log != NULL)
    delete log;
}

/** Torcs section */

void Driver::initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
  this->track = track;
  
  char buffer[256];

 /* switch (s->_raceType) {
        case RM_TYPE_PRACTICE:
            sprintf(buffer, "drivers/frassi/%d/practice/%s", index , track->internalname);
            break;
        case RM_TYPE_QUALIF:
            sprintf(buffer, "drivers/frassi/%d/qualifying/%s", index , track->internalname);
            break;
        case RM_TYPE_RACE:
            sprintf(buffer, "drivers/frassi/%d/race/%s", index , track->internalname);	
            break;
        default:
            break;
  }*/
  
  *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
    
  if (*carParmHandle == NULL) {
    sprintf(buffer, "drivers/frassi/%d/default.xml", index);
    *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
  }

  if(*carParmHandle == NULL)
    cout << "loading default XML " << endl;
  else
    cout << "loading XML:" << buffer << endl;
  
}

void Driver::endrace(int index, tSituation *s){
}

/* Set pitstop commands. */
int Driver::pitCommand(tCarElt* car, tSituation *s){
    return ROB_PIT_IM; /* return immediately */
}

void Driver::newrace(int index, tCarElt* car, tSituation *s){
  
 this->car = car;
 myCar = new carData(car, track);
 trajectory = new trajectoryPlanner(car , myCar , track);
 allCars = new opponents(s , track);
 log = new logger(track, car , myCar , trajectory);
 stuck = 0;
 gearTime = 0.0;
 clutchTime = 0.0;
 
}

void Driver::drive(int index, tSituation *s){ 
    initialUpdate(s);
    
    memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 

    if(myCar->getMode() == STUCK){
      car->ctrl.steer = -stuckangle / car->_steerLock;
      car->ctrl.gear  = -1;
      car->ctrl.accelCmd = 0.65;
      car->ctrl.brakeCmd = 0.0; 
    }   
    else
    {
      double accel = getAccel();
      
      if(accel < 0.0)
      {
	car->ctrl.brakeCmd = filterBrake(fabs(accel));
	car->ctrl.accelCmd = 0.0;
      }
      else
      {
	car->ctrl.brakeCmd = 0.0;
	car->ctrl.accelCmd = filterAccel(filterAccelOpp(accel)); 
      }

      car->ctrl.steer = filterSteer(filterSteerOpp(getSteer()));
      car->ctrl.gear = getGear();  
      car->ctrl.clutchCmd = getClutch();
    }

    finalUpdate(s);
  
}

//TODO stupid check

/** check section */

bool Driver::isStuck(){
	if (fabs(stuckangle) > STUCK_MAX_ANGLE && car->_speed_x < STUCK_MAX_SPEED && fabs(car->_trkPos.toMiddle) > STUCK_MAX_DIST) 
	{
	  if (stuck > STUCK_MAX_COUNT && (stuckangle * car->_trkPos.toMiddle) < 0.0 )
	    return true;
	  else 
	    stuck++;
	} 
	else
	  stuck = 0;

	return false;
}

bool Driver::isRecovery(){  
  if(car->_trkPos.toLeft < -car->_dimension_y || car->_trkPos.toRight < -car->_dimension_y)
    return true;
  else
    return false;  
}

/** update section, there we update all the data of the trajectory, driver, opponents */

void Driver::initialUpdate(tSituation* s){

  myCar->updateCar(s);	//update the car of this driver

  allCars->updateCars(myCar,s);		//update all the opponent 
  allCars->computeStatus(myCar);		//update the status between this driver and all the opponents
  enemyCars = allCars->getEnemyCarsPnt(myCar);	//return a vector of pointer to all the opponent that we don't want to ignore

  trajectory->computeTrajectory(enemyCars);	//compute the trajectory for this driver
  
  stuckangle   = trajectory->getTrajectoryStuck();
  trajangle    = trajectory->getTrajectoryAngle();
  trajdist     = trajectory->getTrajectoryDistance();   
  allowedSpeed = filterSpeed(trajectory->getTrajectorySpeed());
  
  time_difference = s->deltaTime;
  
  gearTime = MAX(0.0 , gearTime - RCM_MAX_DT_ROBOTS);
  
  //TODO we need better algorithm to decide the mode of the car
  
  //TODO code that set the behaviour based on the damage and lap left
  
  if(isStuck())
  {
    myCar->setMode(STUCK);
    return;    
  }
  
  if(isRecovery())
  {
    myCar->setMode(RECOVERY);
    return;    
  } 
  
  //if(car->_gear <= 1 && (car->_speed_x < FILTER_ACCEL_START_MODE_SPEED || myCar->recoveryMode()))
  if(s->currentTime < 10.0 && car->_gear <= 1)
    myCar->setMode(START);
  else 
    //if(car->_gear > 1 || (myCar->recoveryMode() && car->_speed_x >= 5.0))
      myCar->setMode(NORMAL);
    
}

void Driver::finalUpdate(tSituation *s){
  last.allowedSpeed = allowedSpeed;
  last.speed_x = car->_speed_x;
  last.trajangle = trajangle;
  last.carYaw = car->_yaw;
  
  log->log(s);	// log the data

}

/** Controller section, there we compute accel/brake, steer, gear and clutch */

int Driver::getGear(){
  if(gearTime > 0.0)
    return car->_gear;
  
    if (car->_gear <= 0) 
      return 1;
    double omega = car->_enginerpmRedLine/myCar->getGearUp();
    double wr = car->_wheelRadius(REAR_RGT);

    if (omega*wr*GEAR_UP_SPEED_GAIN < car->_speed_x)
    {
	gearTime = GEAR_MAX_TIME;
        return car->_gear + 1;
    }
    else 
    {
      omega = car->_enginerpmRedLine/myCar->getGearDown();
	if (car->_gear > 1 && omega*wr*GEAR_UP_SPEED_GAIN > car->_speed_x + GEAR_DOWN_SPEED_GAIN)
	  return car->_gear - 1;
    }
    return car->_gear;
}

double Driver::getAccel(){

    double error = MIN(ACCEL_CONST_LIMIT,((allowedSpeed - car->_speed_x) + (last.allowedSpeed - last.speed_x))/time_difference);
    double integ = MIN(ACCEL_CONST_LIMIT,((error + last.accelError)*time_difference)/2);

    last.accelErrorSum = MIN(ACCEL_INTEG_LIMIT , last.accelErrorSum + integ);
    last.accelError = error;

    double result = ACCEL_PI_KE*error + ACCEL_PI_KI*last.accelErrorSum;

    result = myCar->getAccel(result); 

    if(result <= 0)
      return MAX(-1.0 , result);
    return MIN(1.0 , result);    
}

double Driver::getSteer(){

  double part_1 = (STEER_K_SPEED*trajdist/ (STEER_K_SOFT + car->_speed_x));

  double O_meas = (car->_yaw  - last.carYaw )/time_difference; 
  double O_traj = (trajangle - last.trajangle)/time_difference;
 
  double part_2 = (O_traj - O_meas);
  
  double O    = trajangle - car->_yaw;
  NORM_PI_PI(O);

  double result = -atan(part_1) + STEER_K_YAW*part_2 + O;

  return result/car->_steerLock;
}

double Driver::getClutch(){

  if (car->_gear > 1) {
    clutchTime = 0.0;
    return 0.0;
  } 
  else 
    {
    double drpm = car->_enginerpm - car->_enginerpmRedLine/2.0;
    clutchTime = MIN(CLUTCH_FULL_MAX_TIME , clutchTime);
    double clutcht = (CLUTCH_FULL_MAX_TIME - clutchTime)/CLUTCH_FULL_MAX_TIME;		
    if (car->_gear == 1 && car->_accelCmd > 0.0) {
      clutchTime += RCM_MAX_DT_ROBOTS;
    }
      if (drpm > 0) {
	double speedr;
	if (car->_gearCmd == 1) {
	  // Compute corresponding speed to engine rpm.
	  double omega = car->_enginerpmRedLine/car->_gearRatio[car->_gear + car->_gearOffset];
	  speedr = (CLUTCH_SPEED + car->_speed_x)/fabs(car->_wheelRadius(REAR_RGT)*omega);
	  double clutchr = MAX(0.0, (1.0 - speedr*2.0*drpm/car->_enginerpmRedLine));
	  return MIN(clutcht, clutchr);
	} 
	else 
	  {
	    // For the reverse gear.
	    clutchTime = 0.0;
	    return 0.0;
	  }
	} else
	    return clutcht;
    }
}

/** opponents section */

double Driver::filterAccelOpp(double accel){
  
  //TODO here we have to reduce accel to avoid collision
  
  return accel;
}

double Driver::filterSteerOpp(double steer){
  
  //TODO here we have to change the steer to avoid collision
  
  return steer;
}

/** Filter section we change we value of the values computed by the controller section with the aim of improving performance */

double Driver::filterBrake(double brake){
  
  if(myCar->recoveryMode())
    brake =  1.0;
  
  if(myCar->getMaxSpeed() != myCar->CAR_MAX_DEFAULT_SPEED)
  {  
  double weight = myCar->getMass()*G;
  double maxForce = weight + myCar->getCA()*myCar->getMaxSpeed()*myCar->getMaxSpeed();
  double force = weight + myCar->getCA()*myCar->getSpeedSqr();
  brake = brake*MIN(1.0, force/maxForce); 
  }
  
  if(car->_speed_x < ABS_MIN_SPEED)
    return brake;
 
  double slip = 0.0;

  for(int i=0; i < 4; i++)
    slip += car->_wheelSpinVel(i) * car->_wheelRadius(i) / car->_speed_x;

  slip /= 4.0;

  if(slip < ABS_SLIP)
    return brake * slip;
  else
    return brake;
}

double Driver::filterAccel(double accel){
  
    if(myCar->getMode() == START)
    {
      double x = car->_gearRatio[car->_gear + car->_gearOffset];
      double omega = (car->_enginerpm / car->_enginerpmRedLine/2)*x/car->_wheelRadius(REAR_RGT); 
      return MAX(FILTER_ACCEL_START , omega);       
    }
    
    if(myCar->recoveryMode())
      accel = accel/(fabs(car->_yaw_rate)*RECOVERY_YAW_RATE) - RECOVERY_ACCEL_REDUCE;

    if (car->_gear <= 1)
      return accel;

    double slip = car->_speed_x - myCar->getDrivenWheelSpeed();
    
    if (slip > TCL_SLIP)
      accel = accel - MIN(accel, (slip - TCL_SLIP)/TCL_RANGE);
    
    return accel;
}

double Driver::filterSteer(double steer){
  
  if(myCar->recoveryMode())
    return steer;

  return steer;
  
}
  
double Driver::filterSpeed(double speed){
   
  if(myCar->recoveryMode())
    return RECOVERY_MAX_SPEED;
  
  return speed;
  
}




