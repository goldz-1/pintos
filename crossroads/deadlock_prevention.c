#include "projects/crossroads/deadlock_prevention.h"
#include "projects/crossroads/priority_sync.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/crossroads.h"
#include "threads/malloc.h"
#include "threads/interrupt.h"
#include <stdio.h>

/* Global deadlock prevention system */
struct deadlock_prevention* deadlock_system = NULL;
struct intersection_safety* safety_system = NULL;

extern int crossroads_step;

void init_deadlock_prevention(void) {
    printf("Initializing simplified deadlock prevention...\n");

    deadlock_system = malloc(sizeof(struct deadlock_prevention));
    if (deadlock_system == NULL) {
        PANIC("Failed to allocate deadlock prevention system");
    }

    /* Initialize zone locks */
    for (int i = 0; i < NUM_ZONES; i++) {
        priority_lock_init(&deadlock_system->zone_locks[i]);
        deadlock_system->zones_occupied[i] = false;
        deadlock_system->zone_holders[i] = 0;
    }

    /* Initialize intersection capacity semaphore with higher capacity */
    priority_sema_init(&deadlock_system->intersection_capacity, 16);

    /* Initialize resource ordering lock */
    lock_init(&deadlock_system->resource_order_lock);

    printf("Deadlock prevention system initialized\n");
}

void init_intersection_safety(void) {
    printf("Initializing intersection safety system...\n");

    safety_system = malloc(sizeof(struct intersection_safety));
    if (safety_system == NULL) {
        PANIC("Failed to allocate intersection safety system");
    }

    /* Initialize safety check lock */
    lock_init(&safety_system->safety_check_lock);

    /* Initialize conflict matrix - simplified for basic implementation */
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            safety_system->conflicting_moves[i][j] = false;
        }
    }

    printf("Intersection safety system initialized\n");
}

void cleanup_deadlock_prevention(void) {
    if (deadlock_system) {
        free(deadlock_system);
        deadlock_system = NULL;
    }
    if (safety_system) {
        free(safety_system);
        safety_system = NULL;
    }
}

int get_zone_for_position(struct position pos) {
    int row = pos.row;
    int col = pos.col;

    /* Only core intersection positions need zone management */
    if (row >= 2 && row <= 4 && col >= 2 && col <= 4) {
        return ZONE_CENTER;
    }

    /* All other positions don't need zone management */
    return -1;
}

bool is_intersection_position(struct position pos) {
    return (pos.row >= 2 && pos.row <= 4 && pos.col >= 2 && pos.col <= 4);
}

int get_movement_direction(struct position from, struct position to) {
    int row_diff = to.row - from.row;
    int col_diff = to.col - from.col;

    if (row_diff > 0 && col_diff == 0) return DIRECTION_NORTH_TO_SOUTH;
    if (row_diff < 0 && col_diff == 0) return DIRECTION_SOUTH_TO_NORTH;
    if (row_diff == 0 && col_diff > 0) return DIRECTION_WEST_TO_EAST;
    if (row_diff == 0 && col_diff < 0) return DIRECTION_EAST_TO_WEST;

    return -1;
}

bool can_enter_intersection(struct vehicle_info* vi, struct position next_pos) {
    if (!deadlock_system) {
        return true;  /* If system not initialized, allow movement */
    }

    /* Only restrict movement into core intersection */
    if (!is_intersection_position(next_pos)) {
        return true;
    }

    /* For ambulances in emergency, always allow */
    if (vi->type == VEHICL_TYPE_AMBULANCE && (vi->golden_time - crossroads_step) <= 3) {
        printf("[DEBUG] Emergency ambulance %c: allowed immediate access\n", vi->id);
        return true;
    }

    /* Simple capacity check - just try to get a slot */
    int priority = get_vehicle_priority(vi);
    if (priority_sema_try_down(&deadlock_system->intersection_capacity, priority)) {
        printf("[DEBUG] %c: acquired intersection capacity\n", vi->id);
        return true;
    }

    printf("[DEBUG] %c: intersection at capacity\n", vi->id);
    return false;
}

bool check_resource_ordering(struct vehicle_info* vi, int required_zones[], int num_zones) {
    return true;  /* Simplified - no complex ordering */
}

bool acquire_zones_atomic(struct vehicle_info* vi, int zones[], int num_zones) {
    return true;  /* Simplified - handled by intersection capacity */
}

void release_zones(struct vehicle_info* vi, int zones[], int num_zones) {
    if (!deadlock_system) return;

    /* Only release if we were in intersection */
    for (int i = 0; i < num_zones; i++) {
        if (zones[i] == ZONE_CENTER) {
            priority_sema_up(&deadlock_system->intersection_capacity);
            printf("[DEBUG] %c: released intersection capacity\n", vi->id);
            break;
        }
    }
}

bool is_safe_movement(struct position from, struct position to, struct vehicle_info* vi) {
    return true;  /* Simplified safety check */
}

bool check_conflicting_paths(struct vehicle_info* vi1, struct vehicle_info* vi2) {
    return false;  /* No conflicts in simplified version */
}

void update_conflict_matrix(void) {
    /* No-op in simplified version */
}

int compare_resource_priority(int zone1, int zone2) {
    return zone1 - zone2;
}

void sort_zones_by_priority(int zones[], int num_zones) {
    /* Simple bubble sort */
    for (int i = 0; i < num_zones - 1; i++) {
        for (int j = 0; j < num_zones - i - 1; j++) {
            if (zones[j] > zones[j + 1]) {
                int temp = zones[j];
                zones[j] = zones[j + 1];
                zones[j + 1] = temp;
            }
        }
    }
}

bool handle_ambulance_priority(struct vehicle_info* vi, struct position next_pos) {
    if (vi->type != VEHICL_TYPE_AMBULANCE) {
        return false;
    }

    int time_left = vi->golden_time - crossroads_step;
    if (time_left <= 3) {
        printf("EMERGENCY: Ambulance %c has priority (time left: %d)\n", vi->id, time_left);
        return true;
    }

    return false;
}

void preempt_normal_vehicles(struct vehicle_info* ambulance) {
    printf("EMERGENCY: Ambulance %c requesting priority access\n", ambulance->id);
}