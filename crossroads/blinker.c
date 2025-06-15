#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/blinker.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/priority_sync.h"
#include "projects/crossroads/deadlock_prevention.h"
#include "projects/crossroads/crossroads.h"
#include "threads/interrupt.h"
#include <stdio.h>

/* Traffic light states */
#define BLINKER_NS_GREEN 0  /* North-South green, East-West red */
#define BLINKER_EW_GREEN 1  /* East-West green, North-South red */
#define BLINKER_ALL_RED  2  /* All red during transition */

/* Minimum green light duration */
#define MIN_GREEN_DURATION 3
#define YELLOW_DURATION 1   /* Transition period */

/* Global blinker variables */
static struct blinker_info* global_blinkers;
static struct priority_lock blinker_control_lock;
static struct priority_condition blinker_change_cond;
static int current_blinker_state = BLINKER_NS_GREEN;
static int green_duration_counter = 0;
static bool blinker_running = false;

/* Thread IDs for blinkers */
static tid_t blinker_threads[NUM_BLINKER];

/* Function prototypes */
static void blinker_thread_func(void* aux);
static bool check_ambulance_needs_priority(struct vehicle_info* vehicles);
static void change_blinker_state(void);
static bool is_safe_to_change_lights(void);
static int get_vehicles_waiting_direction(struct vehicle_info* vehicles, int direction);

void init_blinker(struct blinker_info* blinkers, struct lock** map_locks, struct vehicle_info* vehicle_info) {
    printf("Initializing traffic light system...\n");

    /* Store global references */
    global_blinkers = blinkers;

    /* Initialize priority synchronization primitives */
    priority_lock_init(&blinker_control_lock);
    priority_cond_init(&blinker_change_cond);

    /* Initialize blinker info for each blinker */
    for (int i = 0; i < NUM_BLINKER; i++) {
        blinkers[i].map_locks = map_locks;
        blinkers[i].vehicles = vehicle_info;
    }

    /* Initial state: North-South green */
    current_blinker_state = BLINKER_NS_GREEN;
    green_duration_counter = 0;
    blinker_running = true;

    printf("Traffic light system initialized with NS green\n");
}

void start_blinker() {
    printf("Starting traffic light threads...\n");

    /* Create blinker control threads */
    for (int i = 0; i < NUM_BLINKER; i++) {
        char name[32];
        snprintf(name, sizeof(name), "blinker_%d", i);
        blinker_threads[i] = thread_create(name, PRI_DEFAULT + 1,
            blinker_thread_func, &global_blinkers[i]);
    }

    printf("Traffic light system started\n");
}

static void blinker_thread_func(void* aux) {
    struct blinker_info* blinker = (struct blinker_info*)aux;
    int priority = PRIORITY_TRAFFIC_LIGHT;

    while (blinker_running) {
        priority_lock_acquire(&blinker_control_lock, priority);

        /* Check if we need to handle ambulance priority */
        bool ambulance_urgent = check_ambulance_needs_priority(blinker->vehicles);

        /* Increment duration counter */
        green_duration_counter++;

        /* Determine if we should change lights */
        bool should_change = false;

        if (ambulance_urgent) {
            /* Emergency: Change lights immediately for ambulance */
            should_change = true;
            printf("EMERGENCY: Changing lights for ambulance\n");
        }
        else if (green_duration_counter >= MIN_GREEN_DURATION) {
            /* Check if it's safe and beneficial to change */
            int ns_waiting = get_vehicles_waiting_direction(blinker->vehicles, BLINKER_NS_GREEN);
            int ew_waiting = get_vehicles_waiting_direction(blinker->vehicles, BLINKER_EW_GREEN);

            if (current_blinker_state == BLINKER_NS_GREEN && ew_waiting > ns_waiting) {
                should_change = is_safe_to_change_lights();
            }
            else if (current_blinker_state == BLINKER_EW_GREEN && ns_waiting > ew_waiting) {
                should_change = is_safe_to_change_lights();
            }

            /* Also change if one direction has no waiting vehicles */
            if ((current_blinker_state == BLINKER_NS_GREEN && ns_waiting == 0 && ew_waiting > 0) ||
                (current_blinker_state == BLINKER_EW_GREEN && ew_waiting == 0 && ns_waiting > 0)) {
                should_change = is_safe_to_change_lights();
            }
        }

        if (should_change) {
            change_blinker_state();
        }

        /* Signal waiting vehicles about light state */
        priority_cond_broadcast(&blinker_change_cond, &blinker_control_lock);

        priority_lock_release(&blinker_control_lock);

        /* Wait for next step */
        thread_yield();
    }
}

static bool check_ambulance_needs_priority(struct vehicle_info* vehicles) {
    /* Check all vehicles for urgent ambulances */
    for (int i = 0; vehicles[i].id != 0; i++) {
        if (vehicles[i].type == VEHICL_TYPE_AMBULANCE &&
            vehicles[i].state == VEHICLE_STATUS_RUNNING) {

            int time_left = vehicles[i].golden_time - crossroads_step;

            /* Urgent if less than 3 steps remaining */
            if (time_left <= 3 && time_left > 0) {
                /* Check if ambulance needs different light state */
                int amb_row = vehicles[i].position.row;
                int amb_col = vehicles[i].position.col;

                /* Determine if ambulance is waiting at intersection */
                if ((amb_row == 1 || amb_row == 5) && (amb_col >= 2 && amb_col <= 4)) {
                    /* North or South approach - needs NS green */
                    if (current_blinker_state != BLINKER_NS_GREEN) {
                        return true;
                    }
                }
                else if ((amb_col == 1 || amb_col == 5) && (amb_row >= 2 && amb_row <= 4)) {
                    /* East or West approach - needs EW green */
                    if (current_blinker_state != BLINKER_EW_GREEN) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

static void change_blinker_state(void) {
    /* Toggle between NS and EW green */
    if (current_blinker_state == BLINKER_NS_GREEN) {
        current_blinker_state = BLINKER_EW_GREEN;
        printf("Traffic light changed: East-West GREEN, North-South RED\n");
    }
    else {
        current_blinker_state = BLINKER_NS_GREEN;
        printf("Traffic light changed: North-South GREEN, East-West RED\n");
    }

    /* Reset duration counter */
    green_duration_counter = 0;
}

static bool is_safe_to_change_lights(void) {
    /* Check if any vehicles are currently in the intersection */
    /* This prevents changing lights while vehicles are crossing */

    for (int row = 2; row <= 4; row++) {
        for (int col = 2; col <= 4; col++) {
            if (global_blinkers->map_locks[row][col].holder != NULL) {
                /* Vehicle in intersection - not safe to change */
                return false;
            }
        }
    }

    return true;
}

static int get_vehicles_waiting_direction(struct vehicle_info* vehicles, int direction) {
    int count = 0;

    for (int i = 0; vehicles[i].id != 0; i++) {
        if (vehicles[i].state != VEHICLE_STATUS_RUNNING) continue;

        int row = vehicles[i].position.row;
        int col = vehicles[i].position.col;

        if (direction == BLINKER_NS_GREEN) {
            /* Count vehicles waiting to go North-South */
            if ((row == 1 || row == 5) && (col >= 2 && col <= 4)) {
                count++;
            }
        }
        else {
            /* Count vehicles waiting to go East-West */
            if ((col == 1 || col == 5) && (row >= 2 && row <= 4)) {
                count++;
            }
        }
    }

    return count;
}

/* Public function to check if vehicle can proceed based on traffic light */
bool can_vehicle_proceed(struct position current, struct position next) {
    bool can_proceed = true;

    priority_lock_acquire(&blinker_control_lock, PRIORITY_NORMAL_VEHICLE);

    /* Determine movement direction */
    int row_dif = current.row - next.row;
    int col_dif = current.col - next.col;

    bool is_ns_movement = (current.col == next.col) && (row_dif == 1 || row_dif == -1);
    bool is_ew_movement = (current.row == next.row) && (col_dif == 1 || col_dif == -1);

    /* Check if movement matches current light state */
    if (is_ns_movement && current_blinker_state != BLINKER_NS_GREEN) {
        can_proceed = false;
    }
    else if (is_ew_movement && current_blinker_state != BLINKER_EW_GREEN) {
        can_proceed = false;
    }

    priority_lock_release(&blinker_control_lock);

    return can_proceed;
}

/* Function to wait for green light */
void wait_for_green_light(struct vehicle_info* vi) {
    int priority = get_vehicle_priority(vi);

    priority_lock_acquire(&blinker_control_lock, priority);

    /* Determine which direction the vehicle needs */
    bool needs_ns = false;
    bool needs_ew = false;

    int row = vi->position.row;
    int col = vi->position.col;

    if ((row == 1 || row == 5) && (col >= 2 && col <= 4)) {
        needs_ns = true;
    }
    else if ((col == 1 || col == 5) && (row >= 2 && row <= 4)) {
        needs_ew = true;
    }

    /* Wait until appropriate light is green */
    while ((needs_ns && current_blinker_state != BLINKER_NS_GREEN) ||
        (needs_ew && current_blinker_state != BLINKER_EW_GREEN)) {
        priority_cond_wait(&blinker_change_cond, &blinker_control_lock, priority);
    }

    priority_lock_release(&blinker_control_lock);
}