#ifndef __PROJECTS_PROJECT2_BLINKER_H__
#define __PROJECTS_PROJECT2_BLINKER_H__

/** you can change the number of blinkers */
#define NUM_BLINKER 4

struct blinker_info {
    struct lock **map_locks;
    struct vehicle_info *vehicles;
};

void init_blinker(struct blinker_info* blinkers, struct lock **map_locks, struct vehicle_info * vehicle_info);
void start_blinker();

#endif /* __PROJECTS_PROJECT2_BLINKER_H__ */