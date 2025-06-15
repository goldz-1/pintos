
#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"

static struct lock step_sync_lock;
static struct condition step_sync_cond;
static int vehicles_completed_step = 0;
static int total_active_vehicles = 0;
static bool step_sync_initialized = false;

/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][12] = {
	/* from A */ {
		/* to A */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from B */ {
		/* to A */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from C */ {
		/* to A */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from D */ {
		/* to A */
		{{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	}
};


void parse_vehicles(struct vehicle_info *vehicle_info, char *input)
{
    char *token;
    char *input_copy;
    int vehicle_count = 0;
    
    // copying input
    input_copy = malloc(strlen(input) + 1);
    strcpy(input_copy, input);
    
    // strtok 
    token = strtok(input_copy, ":");
    
    while (token != NULL && vehicle_count < 16) { 
        struct vehicle_info *vi = &vehicle_info[vehicle_count];
        
        vi->id = token[0];
        vi->start = token[1];
        vi->dest = token[2];
        
        // initialize
        vi->state = VEHICLE_STATUS_READY;
        vi->position.row = -1;
        vi->position.col = -1;
        
        // default
        vi->type = VEHICL_TYPE_NORMAL;
        vi->arrival = -1;
        vi->golden_time = -1;
        
        // if ambulance
        if (strlen(token) > 3) {
            char *dot_pos = strchr(token + 3, '.');
            if (dot_pos != NULL) {
                // ambulance
                vi->type = VEHICL_TYPE_AMBULANCE;
                vi->arrival = atoi(token + 3);
                vi->golden_time = atoi(dot_pos + 1);
                
                printf("Ambulance %c: %c->%c, arrival=%d, golden_time=%d\n", 
                       vi->id, vi->start, vi->dest, vi->arrival, vi->golden_time);
            }
        } else {
            printf("Normal vehicle %c: %c->%c\n", vi->id, vi->start, vi->dest);
        }
        
        vehicle_count++;
        token = strtok(NULL, ":");
    }
    
    free(input_copy);
    
    // update global variance
    total_active_vehicles = vehicle_count;
    printf("Total vehicles parsed: %d\n", vehicle_count);
}

static int is_position_outside(struct position pos)
{
    return (pos.row == -1 || pos.col == -1);
}

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
    struct position pos_cur, pos_next;

    pos_next = vehicle_path[start][dest][step];
    pos_cur = vi->position;

    if (vi->state == VEHICLE_STATUS_RUNNING) {
        /* check termination */
        if (is_position_outside(pos_next)) {
            /* actual move */
            vi->position.row = vi->position.col = -1;
            /* release previous */
            lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
            return 0;
        }
    }

    /* lock next position */
    lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
    if (vi->state == VEHICLE_STATUS_READY) {
        /* start this vehicle */
        vi->state = VEHICLE_STATUS_RUNNING;
    } else {
        /* release current position */
        lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
    }
    /* update position */
    vi->position = pos_next;
    
    return 1;
}

static void wait_for_step_completion(void)
{
    lock_acquire(&step_sync_lock);
    
    vehicles_completed_step++;
    
    if (vehicles_completed_step == total_active_vehicles) {
        // if all vehicles are cimpleted -> next
        crossroads_step++;
        vehicles_completed_step = 0;
        
        // call unitstep_changed() -> thread safed
        unitstep_changed();
        
        // calling all waiting vehicles
        cond_broadcast(&step_sync_cond, &step_sync_lock);
    }
    else {
        // waiting for all vehicles are completed
        cond_wait(&step_sync_cond, &step_sync_lock);
    }
    
    lock_release(&step_sync_lock);
}


static bool should_start_vehicle(struct vehicle_info *vi)
{
    if (vi->type == VEHICL_TYPE_NORMAL) {
        return true; // normal 
    }
    
    // ambulance -> starting when designated time
    return crossroads_step >= vi->arrival;
}

static bool check_golden_time(struct vehicle_info *vi)
{
    if (vi->type == VEHICL_TYPE_NORMAL) {
        return true; 
    }
    
    if (crossroads_step > vi->golden_time) {
        printf("AMBULANCE %c MISSED GOLDEN TIME! Current: %d, Deadline: %d\n", 
               vi->id, crossroads_step, vi->golden_time);
        return false;
    }
    
    return true;
}

void init_on_mainthread(int thread_cnt)
{
    if (!step_sync_initialized) {
        lock_init(&step_sync_lock);
        cond_init(&step_sync_cond);
        vehicles_completed_step = 0;
        total_active_vehicles = thread_cnt;
        step_sync_initialized = true;
        
        printf("Step synchronization initialized for %d vehicles\n", thread_cnt);
    }
}

void vehicle_loop(void *_vi)
{
    int res;
    int start, dest, step;
    struct vehicle_info *vi = _vi;

    start = vi->start - 'A';
    dest = vi->dest - 'A';

    vi->position.row = vi->position.col = -1;
    vi->state = VEHICLE_STATUS_READY;

    step = 0;
    
    printf("Vehicle %c starting: %c->%c (type: %s)\n", 
           vi->id, vi->start, vi->dest, 
           vi->type == VEHICL_TYPE_AMBULANCE ? "AMBULANCE" : "NORMAL");

    while (1) {
        // checking starting time
        if (!should_start_vehicle(vi)) {
            wait_for_step_completion();
            continue;
        }
        
        //checking a golden time
        if (!check_golden_time(vi)) {
            // over a golden time
            break;
        }
        
        /* vehicle main code */
        res = try_move(start, dest, step, vi);
        if (res == 1) {
            step++;
            printf("Vehicle %c moved to step %d, position (%d,%d)\n", 
                   vi->id, step, vi->position.row, vi->position.col);
        }

        /* termination condition */ 
        if (res == 0) {
            printf("Vehicle %c reached destination!\n", vi->id);
            break;
        }

        wait_for_step_completion();
    }

    /* status transition must happen before finishing */
    vi->state = VEHICLE_STATUS_FINISHED;
    
    // if finished -> decrease a active vehicle
    lock_acquire(&step_sync_lock);
    total_active_vehicles--;
    
    if (vehicles_completed_step == total_active_vehicles && total_active_vehicles > 0) {
        crossroads_step++;
        vehicles_completed_step = 0;
        unitstep_changed();
        cond_broadcast(&step_sync_cond, &step_sync_lock);
    }
    lock_release(&step_sync_lock);
    
    printf("Vehicle %c finished!\n", vi->id);
}
