/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef OUR_AVOIDER_H
#define OUR_AVOIDER_H
#include <inttypes.h>
#include "math/pprz_geodetic_int.h"

extern uint8_t safeToGoForwards;
extern float incrementForAvoidance;
extern uint16_t trajectoryConfidence;
extern void our_avoider_init(void);
extern void our_avoider_periodic(void);
extern uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
extern uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees);
extern uint8_t chooseRandomIncrementAvoidance(void);
extern uint8_t DetermineIncrementAvoidance(void);

#endif

