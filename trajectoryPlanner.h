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

#ifndef _TRAJECTORYPLANNER_H_
#define _TRAJECTORYPLANNER_H_

#include "carData.h"
#include "opponent.h"

class trajectoryPlanner;

/** a manuever is described by:
 *  - curve with radius R or a straight
 *  - start and final position/angle
 *  - allowed speed
 */

class maneuver {

  public:
    
    maneuver(v2d start, carData *myCar ,double dist , double radius, double angle , double speed , 
	     double cx , double cy, tTrackSeg *startSeg, tTrackSeg *endSeg, trajectoryPlanner *traj);
    
    /** getter */
    
    inline int getType()       	{ return type; }
    inline double getLength()   	{ return length; }
    inline double getRadius()   	{ return radius; }
    inline double getArc()      	{ return arc; }
    inline double getStartAngle()   	{ return startAngle; }
    inline double getEndAngle()      	{ return endAngle; }
    inline double getSpeedSqr()	{ return speedSqr; }
    inline double getSpeed()		{ return speed;	   }
    inline tTrackSeg * getStartSeg()	{ return startSeg; }
    inline tTrackSeg * getEndSeg()	{ return endSeg;   }
    inline v2d getStartPoint() 	{ return startPoint; }
    inline v2d getEndPoint() 		{ return endPoint; }
    
    double getDistance(tCarElt *car);
    double getAngle(tCarElt *car);
    double getAllowedSpeed(carData *myCar);
    double distToEnd(tCarElt *car);
    bool isInside(tCarElt *car);

    /** log methods */
    
    void logManeuver(ofstream &log);

  private:
    
    v2d startPoint , endPoint , center;
    double length , radius , arc , startAngle , endAngle , speed , speedSqr;
    tTrackSeg *startSeg , *endSeg;
    double fx , sx;
    int type;
    
    inline double angleFromStart(tCarElt *car);

};

/** this class works in this way:
 *  
 *  during the init we try to load the graph. If we fail we set the computeTrajectory method to the default function else we use the optimal
 *  function.
 *  the graph is loaded in this way
 *  1 - we load all the nodes from the nodes file
 *  2 - we load all the links form the links file
 *  3 - we load all the manuever in this way :
 * 	3.1 - given a manuever we find the arc where the maneuver is
 * 	3.2 - we find the last known position and angle of the car in this arc (the final point of the last maneuver)
 * 	3.3 - we compute the position of the car at the end of the manuver
 * 	3.4 - we add the manuever to the arc
 *  4 - we find the nearest node and we compute the first Disjktra
 * 
 *  during the execution :
 *  	1 - we find the current arc/maneuver based on the position of the car
 *  	2 - we recompute (if needed) the path
 *	3 - we compute trajectory angle, speed, distance
 * 
 *  the graph works in this way
 *  we use two maps for the nodes
 * 	- the first is used to store the data of the node
 * 	- the second is used by Dijkstra and contains the cost to go to that node from the src node
 * 
 *  Arcs are mapped in this way
 * 	- a map that store the weight (that is the sum of the time of all the maneuver of the arc)
 * 	- a map that store a vector of class maneuver
 * 
 *  For the path we use a structure that is returned by Disjktra (directed path) (it simply stores all the arc from the src to dst)
 * 
 */

class trajectoryPlanner{
  
  public:

    trajectoryPlanner(tCarElt *mycar , carData *myCarData ,  tTrack *mytrack);
    ~trajectoryPlanner();
    
    /** utiliy methods */
   
    double distToSegEnd();
    double getAllowedSpeed(tTrackSeg* seg);
    
    // COMPUTE_TRAJECTORY point at the method used for computing the trajectory 
    // computeTrajectory(..) is only a shortcut to call the method pointed by COMPUTE_TRAJECTORY
    
    void (trajectoryPlanner::*COMPUTE_TRAJECTORY)(vector <opponent *> *);
    inline void computeTrajectory(vector <opponent *> * enemyCars){ (this->*COMPUTE_TRAJECTORY)(enemyCars);}
    
    inline double getTrajectoryAngle() 	{ return angle; }
    inline double getTrajectoryStuck() 	{ return stuckangle; }
    inline double getTrajectorySpeed() 	{ return allowedSpeed; }
    inline double getTrajectoryDistance() 	{ return distance; }
    inline bool   isTrajectoryOptimal()  	{ return trajType == optimalTraj; }
    inline maneuver * getCurrentManeuver()	{ return &arcData[path.nth(pathIndex)][manIndex]; }

    bool isInsideSeg(const v2d point , const tTrackSeg *seg); //true if a point is inside a seg 
    double distFromSegStart(const v2d point , const tTrackSeg *seg); //TODO doesn't work (not sure)
      
    /** log methods */
    
    void logPath(ofstream &log);  //used by logger.cpp
    
  private:
    
    static const double PATH_MAX_DIST;		
    static const double PATH_MAX_STEER;	
    static const double RECOMPUTE_TIME;	/* time that have to pass between two Dijktra recomputation */
    
    static int sectors;		/* number of sectors of the graph */			  
    static int nodePerLine;	/* number of nodes per line  */
    static int nodePerPos ;	/* number of nodes per speed */
    static int nodePerSector;  /* nodePerline * nodePerPos */
    
    bool recomputePath;		/* set true if you want to recompute the optimal path */
    
    tCarElt *car;
    carData *myCar;
    tTrack  *track; 
    double angle , distance , allowedSpeed , stuckangle; /* parameters needed by the controller, computed by computeTrajectory */
    double tangentAngle , recomputeTime;
    int trajType;

    typedef struct nodedata {
      v2d pos;
      double speed, angle , distance;
      int sector;
      tTrackSeg *seg;
      nodedata() : pos() {speed = 0.0; angle = 0.0; sector = 0; seg = NULL; distance = 0.0;}
      nodedata(double x , double y, double speed , double angle , int sector) 
      : pos(x , y) {this->seg = NULL; this->angle = angle; this->speed = speed; this->sector = sector; this->distance = 0.0;}
    } node_data;

    typedef ListDigraph::ArcMap  <double> arcDistMap;			/* map for the weight of the arcs , used by dijktra */
    typedef ListDigraph::ArcMap  <vector <maneuver> > arcDataMap;	/* map that contains for each arc a vector of maneuvers */
    typedef ListDigraph::NodeMap <node_data> nodeDataMap;		/* map that contains all the nodes data */
    typedef ListDigraph::NodeMap <double> nodeDistMap;			/* map filled by dijktra with the cost for each node */
    
    static ListDigraph graph;   		   	/* directed graph , shared between all the drivers */
    arcDistMap arcDist;    		          	/* Map that stores the weight of the arcs */
    arcDistMap arcDistBackUp;			   	/* Map that stores the backUp weight of the arc */	
    arcDataMap arcData;				  	/* Map that stores a vector of maneuvers for each arcs */
    nodeDataMap nodeData;     			   	/* Map that stores attributes for nodes */
    nodeDistMap nodeDist;			  	/* Map that stores the cost to go to that node, return after the execution of Dijkstra */
    ListPath <ListDigraph> path;		   	/* Path to follow , returned by dijkstra. Contains a list of arcs */
    Dijkstra <ListDigraph , arcDistMap> dijkstra;	/* Dijkstra class */
    ArcLookUp <ListDigraph> arcLookup;		  	/* this can be used to retrieve an arc, given src and dst node, in O(log m) time */
     
    int pathIndex;                              	/* index of the current arc , maneuver */
    int manIndex;
    int trajLength;					/* number of maneuvers that compose our path */
    vector <maneuver> traj;			   	/* Pointer to the current vector of maneuvers to follow */
    
    /** utility */
    
    inline int firstIndexOfSector(const node src) { return nodeData[src].sector*nodePerSector; }
    inline int firstIndexOfSector(const int src) {  return src*nodePerSector; } 
    inline int prevSector(const node src) { return (nodeData[src].sector == 0) ? sectors -1 : nodeData[src].sector - 1; }    
    inline int prevSector(const int  src) { return (src == 0) ? sectors -1 : src - 1; }   
    
    inline maneuver * getManeuver(int p, int m) { return &arcData[path.nth(p)][m]; }
    inline maneuver * getManeuver(arc p, int m) { return &arcData[p][m]; }
    inline void incManeuver(int &path , int &man);
    inline void decManeuver(int &p , int &i);
    inline int incPathIndex(int p);
    inline int decPathIndex(int p);
    
    inline bool isBetween(double p , double s , double f) { return (f > s) ? (p > s && p < f) : (p < s && p > f); }
    
    /** optimal methods */
    
    void computeOptimalTrajectory(vector <opponent *> * enemyCars);	/* pointed by COMPUTE_TRAJECTORY */	
    double getOptimalSpeed();						/* compute the allowedSpeed */
    void checkPath();
    void findPosition();
    void computeOptimalPath(node src);
    ListDigraph::Node findNearestNode();
  
    /** initialization mathods */
    
    void initTrajectory();	/* load the trajectory following method */
    
    bool loadNodes();		/* load nodes from path/torcs/track_name/car_name/nodes.csv */
    bool loadManeuvers();	/* load maneuvers from path/torcs/track_name/car_name/maneuvers.csv */
    bool loadLinks();		/* load links from path/torcs/track_name/car_name/links.csv */
    bool initGraph();		/* loadGraph , loadLinks , loadManeuvers , graphExpantion ,init */
    void graphExpansion();	/* extend the graph so we can plan more than one lap */
    ListDigraph::Node initNearestNode();	/* find the nearest node */

    /** default methods for following the raceline , used if initGraph return false */
    
    void computeDefaultTrajectory(vector <opponent *> * enemyCars);
    double getDefaultSpeed(); 

};
 
    
#endif

