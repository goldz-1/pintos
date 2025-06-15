#ifndef __PROJECTS_CROSSROADS_DEADLOCK_PREVENTION_H__
#define __PROJECTS_CROSSROADS_DEADLOCK_PREVENTION_H__

#include "projects/crossroads/position.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/priority_sync.h"
#include "threads/synch.h"

#define ZONE_NORTH_ENTRY    0  // (1,2), (1,3), (1,4)
#define ZONE_SOUTH_ENTRY    1  // (5,2), (5,3), (5,4)  
#define ZONE_WEST_ENTRY     2  // (2,1), (3,1), (4,1)
#define ZONE_EAST_ENTRY     3  // (2,5), (3,5), (4,5)
#define ZONE_CENTER         4  // (2,2), (2,3), (2,4), (3,2), (3,3), (3,4), (4,2), (4,3), (4,4)
#define NUM_ZONES           5

#define DIRECTION_NORTH_TO_SOUTH    0
#define DIRECTION_SOUTH_TO_NORTH    1
#define DIRECTION_WEST_TO_EAST      2
#define DIRECTION_EAST_TO_WEST      3
#define DIRECTION_LEFT_TURN         4
#define DIRECTION_RIGHT_TURN        5
#define DIRECTION_U_TURN           6

struct deadlock_prevention {
    struct priority_lock zone_locks[NUM_ZONES];
    struct priority_sema intersection_capacity;
    struct lock resource_order_lock;
    bool zones_occupied[NUM_ZONES];
    int zone_holders[NUM_ZONES]; 
};

struct intersection_safety {
    bool conflicting_moves[7][7]; 
    struct lock safety_check_lock;
};

extern struct deadlock_prevention *deadlock_system;
extern struct intersection_safety *safety_system;

void init_deadlock_prevention(void);
void init_intersection_safety(void);
void cleanup_deadlock_prevention(void);

int get_zone_for_position(struct position pos);
bool is_intersection_position(struct position pos);
int get_movement_direction(struct position from, struct position to);

bool can_enter_intersection(struct vehicle_info *vi, struct position next_pos);
bool check_resource_ordering(struct vehicle_info *vi, int required_zones[], int num_zones);
bool acquire_zones_atomic(struct vehicle_info *vi, int zones[], int num_zones);
void release_zones(struct vehicle_info *vi, int zones[], int num_zones);

bool is_safe_movement(struct position from, struct position to, struct vehicle_info *vi);
bool check_conflicting_paths(struct vehicle_info *vi1, struct vehicle_info *vi2);
void update_conflict_matrix(void);

int compare_resource_priority(int zone1, int zone2);
void sort_zones_by_priority(int zones[], int num_zones);

bool handle_ambulance_priority(struct vehicle_info *vi, struct position next_pos);
void preempt_normal_vehicles(struct vehicle_info *ambulance);

#endif 