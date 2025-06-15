#ifndef __PROJECTS_PROJECT2_VEHICLE_H__
#define __PROJECTS_PROJECT2_VEHICLE_H__

#include "projects/crossroads/position.h"
#include "threads/synch.h"

/* Vehicle status definitions */
#define VEHICLE_STATUS_READY 	0
#define VEHICLE_STATUS_RUNNING	1
#define VEHICLE_STATUS_FINISHED	2

/* Vehicle type definitions */
#define VEHICL_TYPE_NORMAL 0
#define VEHICL_TYPE_AMBULANCE 1

/* Vehicle information structure */
struct vehicle_info {
	char id;                    
	char state;                 
	char start;                 
	char dest;                  
	
	char type;                  
	char arrival;               
	char golden_time;           
	
	struct position position;   
	struct lock **map_locks;    
};

/* Function declarations */
void vehicle_loop(void *vi);
void parse_vehicles(struct vehicle_info *vehicle_info, char *input);
void init_on_mainthread(int thread_cnt);

/* External path data */
extern const struct position vehicle_path[4][4][12];

#endif /* __PROJECTS_PROJECT2_VEHICLE_H__ */