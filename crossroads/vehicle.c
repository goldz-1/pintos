#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "threads/malloc.h"
#include "threads/interrupt.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"
#include "projects/crossroads/priority_sync.h"
#include "projects/crossroads/deadlock_prevention.h"
#include "projects/crossroads/blinker.h"

static struct lock step_sync_lock;
static struct condition step_sync_cond;
static int vehicles_completed_step = 0;
static int total_active_vehicles = 0;
static bool step_sync_initialized = false;
static int total_vehicle_count = 0;

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

void parse_vehicles(struct vehicle_info* vehicle_info, char* input)
{
    char* token;
    char* input_copy;
    char* saveptr;
    int vehicle_count = 0;

    /* Make a copy of input string */
    input_copy = malloc(strlen(input) + 1);
    strlcpy(input_copy, input, strlen(input) + 1);

    /* Parse each vehicle using strtok_r */
    token = strtok_r(input_copy, ":", &saveptr);

    while (token != NULL && vehicle_count < 16) {
        struct vehicle_info* vi = &vehicle_info[vehicle_count];

        /* Basic vehicle info */
        vi->id = token[0];
        vi->start = token[1];
        vi->dest = token[2];

        /* Initialize state */
        vi->state = VEHICLE_STATUS_READY;
        vi->position.row = -1;
        vi->position.col = -1;

        /* Default to normal vehicle */
        vi->type = VEHICL_TYPE_NORMAL;
        vi->arrival = 0;
        vi->golden_time = -1;

        /* Check if ambulance (has timing info) */
        if (strlen(token) > 3) {
            char* dot_pos = strchr(token + 3, '.');
            if (dot_pos != NULL) {
                /* Parse ambulance timing */
                vi->type = VEHICL_TYPE_AMBULANCE;
                vi->arrival = atoi(token + 3);
                vi->golden_time = atoi(dot_pos + 1);

                printf("Ambulance %c: %c->%c, arrival=%d, golden_time=%d\n",
                    vi->id, vi->start, vi->dest, vi->arrival, vi->golden_time);
            }
        }
        else {
            printf("Normal vehicle %c: %c->%c\n", vi->id, vi->start, vi->dest);
        }

        vehicle_count++;
        token = strtok_r(NULL, ":", &saveptr);
    }

    free(input_copy);

    /* Update global counters */
    total_active_vehicles = vehicle_count;
    total_vehicle_count = vehicle_count;
    printf("Total vehicles parsed: %d\n", vehicle_count);
}

static int is_position_outside(struct position pos)
{
    return (pos.row == -1 || pos.col == -1);
}

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info* vi)
{
    struct position pos_cur, pos_next;
    int current_zone = -1, next_zone = -1;
    bool is_intersection_move = false;

    pos_next = vehicle_path[start][dest][step];
    pos_cur = vi->position;

    if (vi->state == VEHICLE_STATUS_RUNNING) {
        /* Check termination */
        if (is_position_outside(pos_next)) {
            /* Release current position */
            vi->position.row = vi->position.col = -1;

            current_zone = get_zone_for_position(pos_cur);
            if (current_zone != -1) {
                int zones[] = { current_zone };
                release_zones(vi, zones, 1);
                priority_sema_up(&deadlock_system->intersection_capacity);
            }
            else {
                /* Release lock for non-intersection areas */
                lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
            }
            return 0;
        }
    }

    /* Check if entering intersection */
    next_zone = get_zone_for_position(pos_next);
    is_intersection_move = (next_zone != -1);

    if (is_intersection_move) {
        /* Use deadlock prevention system for intersection */
        if (!can_enter_intersection(vi, pos_next)) {
            return -1;
        }

        /* Try to acquire map lock */
        if (vi->type == VEHICL_TYPE_AMBULANCE &&
            (vi->golden_time - crossroads_step) <= 2) {
            /* Emergency ambulance forces entry */
            lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
        }
        else {
            if (!lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])) {
                /* Failed to get lock, release zone */
                int zones[] = { next_zone };
                release_zones(vi, zones, 1);
                priority_sema_up(&deadlock_system->intersection_capacity);
                return -1;
            }
        }

        /* Update vehicle state */
        if (vi->state == VEHICLE_STATUS_READY) {
            vi->state = VEHICLE_STATUS_RUNNING;
        }
        else {
            /* Release previous position */
            current_zone = get_zone_for_position(pos_cur);
            if (current_zone != -1 && current_zone != next_zone) {
                int zones[] = { current_zone };
                release_zones(vi, zones, 1);
            }
            else if (current_zone == -1) {
                lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
            }
        }
    }
    else {
        /* Non-intersection movement */
        if (vi->type == VEHICL_TYPE_AMBULANCE &&
            (vi->golden_time - crossroads_step) <= 2) {
            /* Emergency ambulance */
            lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
        }
        else {
            if (!lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])) {
                return -1;
            }
        }

        if (vi->state == VEHICLE_STATUS_READY) {
            vi->state = VEHICLE_STATUS_RUNNING;
        }
        else {
            /* Release previous position */
            current_zone = get_zone_for_position(pos_cur);
            if (current_zone != -1) {
                int zones[] = { current_zone };
                release_zones(vi, zones, 1);
                priority_sema_up(&deadlock_system->intersection_capacity);
            }
            else {
                lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
            }
        }
    }

    /* Update position */
    vi->position = pos_next;

    return 1;
}

static void wait_for_step_completion(void)
{
    lock_acquire(&step_sync_lock);

    vehicles_completed_step++;

    if (vehicles_completed_step >= total_active_vehicles) {
        /* All active vehicles completed their step */
        crossroads_step++;
        vehicles_completed_step = 0;

        /* Call unitstep_changed() with thread safety */
        lock_release(&step_sync_lock);
        unitstep_changed();
        lock_acquire(&step_sync_lock);

        /* Wake up all waiting vehicles */
        cond_broadcast(&step_sync_cond, &step_sync_lock);
    }
    else {
        /* Wait for other vehicles */
        cond_wait(&step_sync_cond, &step_sync_lock);
    }

    lock_release(&step_sync_lock);
}

static bool should_start_vehicle(struct vehicle_info* vi)
{
    if (vi->type == VEHICL_TYPE_NORMAL) {
        return true;
    }

    /* Ambulance starts at specified time */
    if (vi->type == VEHICL_TYPE_AMBULANCE) {
        return crossroads_step >= vi->arrival;
    }

    return false;
}

static void handle_ambulance_waiting(struct vehicle_info* vi)
{
    if (vi->type == VEHICL_TYPE_AMBULANCE && crossroads_step < vi->arrival) {
        int wait_time = vi->arrival - crossroads_step;
        if (wait_time <= 3) {
            printf("AMBULANCE %c STANDBY - %d steps until dispatch\n",
                vi->id, wait_time);
        }
    }
}

static bool check_golden_time(struct vehicle_info* vi)
{
    if (vi->type == VEHICL_TYPE_NORMAL) {
        return true;
    }

    if (crossroads_step > vi->golden_time) {
        printf("AMBULANCE %c FAILED - Missed golden time!\n", vi->id);
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
        total_vehicle_count = thread_cnt;
        step_sync_initialized = true;

        /* Initialize deadlock prevention systems */
        init_deadlock_prevention();
        init_intersection_safety();

        printf("Step synchronization initialized for %d vehicles\n", thread_cnt);
    }
}

void vehicle_loop(void* _vi)
{
    int res;
    int start, dest, step;
    struct vehicle_info* vi = _vi;

    start = vi->start - 'A';
    dest = vi->dest - 'A';

    vi->position.row = vi->position.col = -1;
    vi->state = VEHICLE_STATUS_READY;

    step = 0;

    printf("Vehicle %c thread started: %c->%c (type: %s)\n",
        vi->id, vi->start, vi->dest,
        vi->type == VEHICL_TYPE_AMBULANCE ? "AMBULANCE" : "NORMAL");

    while (1) {
        /* Check if vehicle should start */
        if (!should_start_vehicle(vi)) {
            handle_ambulance_waiting(vi);
            wait_for_step_completion();
            continue;
        }

        /* Announce ambulance dispatch */
        if (vi->state == VEHICLE_STATUS_READY && step == 0 &&
            vi->type == VEHICL_TYPE_AMBULANCE) {
            printf("AMBULANCE %c DISPATCHED at step %d\n",
                vi->id, crossroads_step);
        }

        /* Check golden time */
        if (!check_golden_time(vi)) {
            break;
        }

        /* Try to move */
        res = try_move(start, dest, step, vi);

        if (res == 1) {
            /* Successfully moved */
            step++;
            if (vi->type == VEHICL_TYPE_AMBULANCE) {
                int time_left = vi->golden_time - crossroads_step;
                if (time_left <= 3) {
                    printf("AMBULANCE %c URGENT - %d steps left!\n",
                        vi->id, time_left);
                }
            }
        }

        /* Check termination */
        if (res == 0) {
            if (vi->type == VEHICL_TYPE_AMBULANCE) {
                if (crossroads_step <= vi->golden_time) {
                    printf("AMBULANCE %c SUCCESS - Arrived in time!\n", vi->id);
                }
                else {
                    printf("AMBULANCE %c FAILED - Arrived too late!\n", vi->id);
                }
            }
            else {
                printf("Vehicle %c arrived at destination\n", vi->id);
            }
            break;
        }

        /* Wait for next step */
        wait_for_step_completion();
    }

    /* Mark as finished */
    vi->state = VEHICLE_STATUS_FINISHED;

    /* Decrement active vehicles */
    lock_acquire(&step_sync_lock);
    total_active_vehicles--;

    /* Check if we need to advance step */
    if (vehicles_completed_step >= total_active_vehicles && total_active_vehicles > 0) {
        crossroads_step++;
        vehicles_completed_step = 0;
        lock_release(&step_sync_lock);
        unitstep_changed();
        lock_acquire(&step_sync_lock);
        cond_broadcast(&step_sync_cond, &step_sync_lock);
    }
    lock_release(&step_sync_lock);

    printf("Vehicle %c thread finished\n", vi->id);
}