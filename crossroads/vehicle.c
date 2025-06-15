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

/* Check if vehicle needs traffic light permission */
static bool needs_traffic_light_check(struct position current, struct position next) {
    /* Only check when entering intersection from outside */
    bool current_in_intersection = (current.row >= 2 && current.row <= 4 && current.col >= 2 && current.col <= 4);
    bool next_in_intersection = (next.row >= 2 && next.row <= 4 && next.col >= 2 && next.col <= 4);

    /* Need to check if moving into intersection from outside */
    return (!current_in_intersection && next_in_intersection);
}

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info* vi)
{
    struct position pos_cur, pos_next;
    bool was_in_intersection = false;
    bool will_be_in_intersection = false;

    pos_next = vehicle_path[start][dest][step];
    pos_cur = vi->position;

    /* Check for termination */
    if (vi->state == VEHICLE_STATUS_RUNNING) {
        if (is_position_outside(pos_next)) {
            /* Vehicle reaches destination */
            was_in_intersection = is_intersection_position(pos_cur);

            if (was_in_intersection) {
                int zones[] = { ZONE_CENTER };
                release_zones(vi, zones, 1);
            }

            if (!is_position_outside(pos_cur)) {
                /* Release map lock */
                if (vi->map_locks[pos_cur.row][pos_cur.col].holder == thread_current()) {
                    lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
                }
            }
            vi->position.row = vi->position.col = -1;
            return 0;
        }
    }

    /* Check traffic light if needed */
    if (vi->state == VEHICLE_STATUS_RUNNING &&
        needs_traffic_light_check(pos_cur, pos_next)) {
        if (!can_vehicle_proceed(pos_cur, pos_next)) {
            printf("VEHICLE %c waiting: red light at (%d,%d) -> (%d,%d) step %d\n",
                vi->id, pos_cur.row, pos_cur.col, pos_next.row, pos_next.col, crossroads_step);
            return -1;  /* Wait for green light */
        }
    }

    will_be_in_intersection = is_intersection_position(pos_next);

    /* Check intersection entry restrictions */
    if (will_be_in_intersection && vi->state == VEHICLE_STATUS_RUNNING) {
        if (!is_intersection_position(pos_cur)) {
            /* Entering intersection from outside */
            if (!can_enter_intersection(vi, pos_next)) {
                return -1;
            }
        }
    }

    /* Try to acquire map lock for next position */
    if (vi->type == VEHICL_TYPE_AMBULANCE && (vi->golden_time - crossroads_step) <= 2) {
        /* Emergency ambulance - blocking acquire */
        lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
    }
    else {
        /* Try non-blocking acquire */
        if (!lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])) {
            /* Failed to get position lock */
            if (will_be_in_intersection && !is_intersection_position(pos_cur)) {
                /* Release intersection capacity if we just acquired it */
                int zones[] = { ZONE_CENTER };
                release_zones(vi, zones, 1);
            }
            return -1;
        }
    }

    /* Successfully acquired new position, release old position */
    if (vi->state == VEHICLE_STATUS_READY) {
        vi->state = VEHICLE_STATUS_RUNNING;
    }
    else if (!is_position_outside(pos_cur)) {
        was_in_intersection = is_intersection_position(pos_cur);

        /* Release old intersection capacity if leaving intersection */
        if (was_in_intersection && !will_be_in_intersection) {
            int zones[] = { ZONE_CENTER };
            release_zones(vi, zones, 1);
        }

        /* Release map lock for old position */
        if (vi->map_locks[pos_cur.row][pos_cur.col].holder == thread_current()) {
            lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
        }
    }

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

        if (res == -1) {
            printf("Vehicle %c blocked at step %d\n", vi->id, step);
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