/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#include "modules/our_avoider/our_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "state.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "modules/computer_vision/cv_opencvdemo2.h"
#include "modules/computer_vision/opencv_ourmainf.h"



#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif



uint8_t safeToGoForwards        = false;
//int memoR = 0;
//int memoL = 0;
//int tresholdColorCount          = 0.05 * 124800; // 520 x 240 = 124.800 total pixels
float tresholdColorCount          = 0.9;
float incrementForAvoidance;
uint16_t trajectoryConfidence   = 1;
float maxDistance               = 1.5;
int turnrate = 5;
//int count_time;
/*
 * Initialisation function, setting the colour filter, random seed and incrementForAvoidance
 */
void our_avoider_init()
{
  // Initialise the variables of the colorfilter to accept orange
  // Initialise random values
  srand(time(NULL));
//  count_time = 0;
  chooseRandomIncrementAvoidance();
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void our_avoider_periodic()
{
//	count_time++;
  // Check the amount of orange. If this is above a threshold
  // you want to turn a certain amount of degrees
  safeToGoForwards = color_count > tresholdColorCount;
  uint8_t trajectoryConfidenceLeap[9] = {0,1,5,10,20,40,60,90,120};
  VERBOSE_PRINT("Color_count: %f  \n", color_count, tresholdColorCount, safeToGoForwards);
  printf("Color_count: %f  \n", color_count, tresholdColorCount, safeToGoForwards);
  VERBOSE_PRINT("trajectoryConfidence: %d   \n",trajectoryConfidence);
  double moveDistance = fmin(maxDistance, 0.0125 * trajectoryConfidenceLeap[trajectoryConfidence]);

  waypoint_set_here_2d(WP_TRAJECTORY0);
  moveWaypointForward(WP_TRAJECTORY0, 0.9);


  if (safeToGoForwards) {
//	  VERBOSE_PRINT("Color_count: %f  threshold: %f safe: %d \n", color_count, tresholdColorCount, safeToGoForwards);
    moveWaypointForward(WP_GOAL, moveDistance);
    moveWaypointForward(WP_TRAJECTORY, 1.25 * moveDistance);
    nav_set_heading_towards_waypoint(WP_GOAL);

    VERBOSE_PRINT("distance next way point: %f   \n",moveDistance);

    // Determine heading
    DetermineIncrementAvoidance();
    if (trajectoryConfidence<8)
    {
      trajectoryConfidence += 1;
    }
    
  } else {
//	  VERBOSE_PRINT("Color_count: %f !!!!!!!!!!! STOP !!!!!!!!!!", color_count, tresholdColorCount, safeToGoForwards);


    waypoint_set_here_2d(WP_GOAL);
    waypoint_set_here_2d(WP_TRAJECTORY);

    increase_nav_heading(&nav_heading, incrementForAvoidance);
    if (trajectoryConfidence>0)
    {
      trajectoryConfidence/=2;
    }
    
//    if (incrementForAvoidance > 0) {
//    	//turning to the left
//    	memoL = count_time;
//    } else {
//    	memoR = count_time;
//    }
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees)
{
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL(RadOfDeg(incrementDegrees));
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
  *heading = newHeading;
  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(ANGLE_FLOAT_OF_BFP(*heading)));
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y),
                DegOfRad(ANGLE_FLOAT_OF_BFP(eulerAngles->psi)) );
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance()
{
  // Randomly choose CW or CCW avoiding direction
  int r = rand() % 2;
  if (r == 0) {
    incrementForAvoidance = 10.0;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  } else {
    incrementForAvoidance = -10.0;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  }
  return false;
}


/*
 * Sets the variable 'incrementForAvoidance' positive/negative based on the side counters
 */
uint8_t DetermineIncrementAvoidance()
{

  if ((color_count_right < color_count_left ) ){
    incrementForAvoidance = 15.0;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  } else if ((color_count_right > color_count_left ) ) {
    incrementForAvoidance = -20.0;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  }




  return false;
}

