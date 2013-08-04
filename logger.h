/***************************************************************************

    file                 : logger.h
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

#ifndef _LOGGER_H_
#define _LOGGER_H_

#include "variables.h"
#include "trajectoryPlanner.h"
#include "carData.h"

class logger {

  public:
    
    static const bool LOG_MATLAB_DATA;		 /* log the track and car data , used by matlab */
    static const bool LOG_CAR_DATA;    	 /* log beavhiour of the car */
    static const bool LOG_TRACK_GNUPLOT;	 /* log the track in gnuplot format */
    static const bool LOG_PATH_GNUPLOT;	 /* log the path in gnuplot format */
    static const bool LOG_CUSTOM_DATA;		 /* log custom data */
    static const int  LOG_STEP;		 /* step * 0.02 sec */

    logger(tTrack *track, tCarElt *car , carData *myCar, trajectoryPlanner *trajectory);
    ~logger();
    
    void log(tSituation *s);

  private:
    
    tTrack *track;
    tCarElt *car;
    carData *myCar;
    trajectoryPlanner *trajectory;
    
    ofstream matlabLogger, pathLogger, trackLogger , carLogger, customLogger;
    double lastSpeed, lastTime;
    int step;

    void logCarData(tSituation *s);
    void logMatlabData();
    void logGnuplotTrack();
    void logGnuplotPath();
    void logCustomData(tSituation *s);
    
};


#endif

