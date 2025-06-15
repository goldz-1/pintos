#ifndef __PROJECTS_PROJECT2_BLINKER_H__
#define __PROJECTS_PROJECT2_BLINKER_H__

#include "projects/crossroads/position.h"
#include "projects/crossroads/vehicle.h"

/** you can change the number of blinkers */
#define NUM_BLINKER 4

struct blinker_info {
    struct lock **map_locks;
    struct vehicle_info *vehicles;
};

void init_blinker(struct blinker_info* blinkers, struct lock **map_locks, struct vehicle_info * vehicle_info);
void start_blinker(void);

/* Additional functions for traffic light control */
bool can_vehicle_proceed(struct position current, struct position next);
void wait_for_green_light(struct vehicle_info *vi);

#endif /* __PROJECTS_PROJECT2_BLINKER_H__ */