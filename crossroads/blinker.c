#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/blinker.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/crossroads.h"
#include "threads/interrupt.h"
#include <stdio.h>

/* Traffic light states */
#define BLINKER_NS_GREEN 0  /* North-South green, East-West red */
#define BLINKER_EW_GREEN 1  /* East-West green, North-South red */

/* Global blinker variables */
static struct blinker_info* global_blinkers;
static struct lock blinker_control_lock;
static int current_blinker_state = BLINKER_NS_GREEN;
static int step_counter = 0;
static bool blinker_running = false;

/* Thread IDs for blinkers */
static tid_t blinker_threads[NUM_BLINKER];

/* Function prototypes */
static void blinker_thread_func(void* aux);

void init_blinker(struct blinker_info* blinkers, struct lock** map_locks, struct vehicle_info* vehicle_info) {
    printf("Initializing simplified traffic light system...\n");

    /* Store global references */
    global_blinkers = blinkers;

    /* Initialize synchronization primitives */
    lock_init(&blinker_control_lock);

    /* Initialize blinker info for each blinker */
    for (int i = 0; i < NUM_BLINKER; i++) {
        blinkers[i].map_locks = map_locks;
        blinkers[i].vehicles = vehicle_info;
    }

    /* Initial state: North-South green */
    current_blinker_state = BLINKER_NS_GREEN;
    step_counter = 0;
    blinker_running = true;

    printf("Traffic light system initialized with NS green\n");
}

void start_blinker() {
    printf("Starting simplified traffic light...\n");

    /* Create only one blinker control thread */
    blinker_threads[0] = thread_create("traffic_light", PRI_DEFAULT + 1,
        blinker_thread_func, &global_blinkers[0]);

    printf("Traffic light system started\n");
}

static void blinker_thread_func(void* aux) {
    extern int crossroads_step;

    while (blinker_running) {
        lock_acquire(&blinker_control_lock);

        /* Simple time-based switching every 3 steps */
        if (crossroads_step > 0 && crossroads_step % 3 == 0 && step_counter != crossroads_step) {
            if (current_blinker_state == BLINKER_NS_GREEN) {
                current_blinker_state = BLINKER_EW_GREEN;
                printf("Traffic light: East-West GREEN, North-South RED (step %d)\n", crossroads_step);
            }
            else {
                current_blinker_state = BLINKER_NS_GREEN;
                printf("Traffic light: North-South GREEN, East-West RED (step %d)\n", crossroads_step);
            }
            step_counter = crossroads_step;
        }

        lock_release(&blinker_control_lock);

        /* Yield to other threads */
        thread_yield();
    }
}

/* Public function to check if vehicle can proceed based on traffic light */
bool can_vehicle_proceed(struct position current, struct position next) {
    bool can_proceed = true;

    lock_acquire(&blinker_control_lock);

    /* Determine movement direction */
    int row_diff = next.row - current.row;
    int col_diff = next.col - current.col;

    bool is_ns_movement = (current.col == next.col) && (row_diff != 0);
    bool is_ew_movement = (current.row == next.row) && (col_diff != 0);

    /* Check if movement matches current light state */
    if (is_ns_movement && current_blinker_state != BLINKER_NS_GREEN) {
        can_proceed = false;
    }
    else if (is_ew_movement && current_blinker_state != BLINKER_EW_GREEN) {
        can_proceed = false;
    }

    lock_release(&blinker_control_lock);

    return can_proceed;
}

/* Function to wait for green light - simplified version */
void wait_for_green_light(struct vehicle_info* vi) {
    /* In simplified version, just return - vehicle will retry next step */
    return;
}