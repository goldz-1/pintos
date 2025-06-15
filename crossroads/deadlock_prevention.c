#include "projects/crossroads/deadlock_prevention.h"
#include "projects/crossroads/crossroads.h"
#include "projects/crossroads/blinker.h"
#include "threads/malloc.h"
#include <stdio.h>

struct deadlock_prevention* deadlock_system = NULL;
struct intersection_safety* safety_system = NULL;

void init_deadlock_prevention(void)
{
    deadlock_system = malloc(sizeof(struct deadlock_prevention));
    ASSERT(deadlock_system != NULL);

    for (int i = 0; i < NUM_ZONES; i++) {
        priority_lock_init(&deadlock_system->zone_locks[i]);
        deadlock_system->zones_occupied[i] = false;
        deadlock_system->zone_holders[i] = -1;
    }

    /* Allow multiple vehicles in intersection but limit total capacity */
    /* This prevents the intersection from being used by only one vehicle at a time */
    priority_sema_init(&deadlock_system->intersection_capacity, 4);

    lock_init(&deadlock_system->resource_order_lock);

    printf("Deadlock prevention system initialized\n");
}

void init_intersection_safety(void)
{
    safety_system = malloc(sizeof(struct intersection_safety));
    ASSERT(safety_system != NULL);

    lock_init(&safety_system->safety_check_lock);

    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            safety_system->conflicting_moves[i][j] = false;
        }
    }

    update_conflict_matrix();

    printf("Intersection safety system initialized\n");
}

void cleanup_deadlock_prevention(void)
{
    if (deadlock_system) {
        free(deadlock_system);
        deadlock_system = NULL;
    }
    if (safety_system) {
        free(safety_system);
        safety_system = NULL;
    }
}

int get_zone_for_position(struct position pos)
{
    if (pos.row == 1 && (pos.col >= 2 && pos.col <= 4)) {
        return ZONE_NORTH_ENTRY;
    }
    if (pos.row == 5 && (pos.col >= 2 && pos.col <= 4)) {
        return ZONE_SOUTH_ENTRY;
    }
    if ((pos.row >= 2 && pos.row <= 4) && pos.col == 1) {
        return ZONE_WEST_ENTRY;
    }
    if ((pos.row >= 2 && pos.row <= 4) && pos.col == 5) {
        return ZONE_EAST_ENTRY;
    }
    if ((pos.row >= 2 && pos.row <= 4) && (pos.col >= 2 && pos.col <= 4)) {
        return ZONE_CENTER;
    }

    return -1;
}

bool is_intersection_position(struct position pos)
{
    return get_zone_for_position(pos) != -1;
}

int get_movement_direction(struct position from, struct position to)
{
    if (from.row > to.row) return DIRECTION_SOUTH_TO_NORTH;
    if (from.row < to.row) return DIRECTION_NORTH_TO_SOUTH;
    if (from.col > to.col) return DIRECTION_EAST_TO_WEST;
    if (from.col < to.col) return DIRECTION_WEST_TO_EAST;
    return -1;
}

bool can_enter_intersection(struct vehicle_info* vi, struct position next_pos)
{
    int zone = get_zone_for_position(next_pos);
    if (zone == -1) return true;

    /* Check traffic light before entering intersection */
    if (!can_vehicle_proceed(vi->position, next_pos)) {
        if (vi->type == VEHICL_TYPE_AMBULANCE) {
            int time_left = vi->golden_time - crossroads_step;
            if (time_left <= 2) {
                printf("AMBULANCE %c OVERRIDING red light!\n", vi->id);
                return true;
            }
            else {
                return false;
            }
        }
        else {
            printf("VEHICLE %c waiting: red light at (%d,%d) ¡æ (%d,%d) step %d\n",
                vi->id, vi->position.row, vi->position.col, next_pos.row, next_pos.col, crossroads_step);
            return false;
        }
    }


    if (vi->type == VEHICL_TYPE_AMBULANCE) {
        return handle_ambulance_priority(vi, next_pos);
    }

    if (!is_safe_movement(vi->position, next_pos, vi)) {
        printf("[DEBUG] %c: unsafe movement from (%d,%d) to (%d,%d)\n",
            vi->id, vi->position.row, vi->position.col, next_pos.row, next_pos.col);
        return false;
    }

    int priority = get_vehicle_priority(vi);
    bool sema_try = priority_sema_try_down(&deadlock_system->intersection_capacity, priority);
    if (!sema_try) {
        printf("[DEBUG] %c: intersection capacity full\n", vi->id);
        return false;
    }


    bool lock_try = priority_lock_try_acquire(&deadlock_system->zone_locks[zone], priority);
    if (!lock_try) {
        priority_sema_up(&deadlock_system->intersection_capacity);
        printf("[DEBUG] %c: failed to acquire zone lock %d\n", vi->id, zone);
        return false;
    }

    deadlock_system->zones_occupied[zone] = true;
    deadlock_system->zone_holders[zone] = vi->id;

    return true;
}

bool check_resource_ordering(struct vehicle_info* vi, int required_zones[], int num_zones)
{
    sort_zones_by_priority(required_zones, num_zones);

    for (int i = 0; i < num_zones; i++) {
        int zone = required_zones[i];
        if (deadlock_system->zones_occupied[zone] &&
            deadlock_system->zone_holders[zone] != vi->id) {
            return false;
        }
    }

    return true;
}

bool acquire_zones_atomic(struct vehicle_info* vi, int zones[], int num_zones)
{
    int priority = get_vehicle_priority(vi);
    bool success = true;

    lock_acquire(&deadlock_system->resource_order_lock);

    for (int i = 0; i < num_zones; i++) {
        if (!priority_lock_try_acquire(&deadlock_system->zone_locks[zones[i]], priority)) {
            success = false;
            break;
        }
    }

    if (!success) {
        for (int i = 0; i < num_zones; i++) {
            priority_lock_release(&deadlock_system->zone_locks[zones[i]]);
        }
    }
    else {
        for (int i = 0; i < num_zones; i++) {
            deadlock_system->zones_occupied[zones[i]] = true;
            deadlock_system->zone_holders[zones[i]] = vi->id;
        }
    }

    lock_release(&deadlock_system->resource_order_lock);
    return success;
}

void release_zones(struct vehicle_info* vi, int zones[], int num_zones)
{
    lock_acquire(&deadlock_system->resource_order_lock);

    for (int i = 0; i < num_zones; i++) {
        if (deadlock_system->zone_holders[zones[i]] == vi->id) {
            deadlock_system->zones_occupied[zones[i]] = false;
            deadlock_system->zone_holders[zones[i]] = -1;
            priority_lock_release(&deadlock_system->zone_locks[zones[i]]);
        }
    }

    lock_release(&deadlock_system->resource_order_lock);
}


bool is_safe_movement(struct position from, struct position to, struct vehicle_info* vi)
{
    lock_acquire(&safety_system->safety_check_lock);

    bool safe = true;

    int from_zone = get_zone_for_position(from);
    int to_zone = get_zone_for_position(to);

    bool emergency_override = false;
    if (vi->type == VEHICL_TYPE_AMBULANCE && (vi->golden_time - crossroads_step <= 2)) {
        emergency_override = true;
    }

    printf("(%d, %d)", from_zone, to_zone);

    if (to_zone != -1) {
        lock_release(&safety_system->safety_check_lock);
        return true;
    }


    for (int i = 0; i < NUM_ZONES; i++) {
        if (deadlock_system->zones_occupied[i] &&
            deadlock_system->zone_holders[i] != vi->id) {

            bool conflict = safety_system->conflicting_moves[to_zone][i];

            if (conflict && emergency_override) {
                printf("AMBULANCE %c OVERRIDING conflict: zone %d vs %d\n", vi->id, to_zone, i);
                continue;
            }

            if (conflict) {
                safe = false;
                break;
            }
        }
    }

    lock_release(&safety_system->safety_check_lock);
    return safe;
}

bool check_conflicting_paths(struct vehicle_info* vi1, struct vehicle_info* vi2)
{
    if (vi1->position.row == vi2->position.row &&
        vi1->position.col == vi2->position.col) {
        return true;
    }

    int dir1 = get_movement_direction(vi1->position, vi1->position);
    int dir2 = get_movement_direction(vi2->position, vi2->position);

    if ((dir1 == DIRECTION_NORTH_TO_SOUTH && dir2 == DIRECTION_WEST_TO_EAST) ||
        (dir1 == DIRECTION_WEST_TO_EAST && dir2 == DIRECTION_NORTH_TO_SOUTH)) {
        return true;
    }

    return false;
}

void update_conflict_matrix(void)
{
    /* North-South vs East-West movements conflict */
    safety_system->conflicting_moves[ZONE_NORTH_ENTRY][ZONE_WEST_ENTRY] = true;
    safety_system->conflicting_moves[ZONE_WEST_ENTRY][ZONE_NORTH_ENTRY] = true;

    safety_system->conflicting_moves[ZONE_NORTH_ENTRY][ZONE_EAST_ENTRY] = true;
    safety_system->conflicting_moves[ZONE_EAST_ENTRY][ZONE_NORTH_ENTRY] = true;

    safety_system->conflicting_moves[ZONE_SOUTH_ENTRY][ZONE_WEST_ENTRY] = true;
    safety_system->conflicting_moves[ZONE_WEST_ENTRY][ZONE_SOUTH_ENTRY] = true;

    safety_system->conflicting_moves[ZONE_SOUTH_ENTRY][ZONE_EAST_ENTRY] = true;
    safety_system->conflicting_moves[ZONE_EAST_ENTRY][ZONE_SOUTH_ENTRY] = true;

    /* Center zone conflicts with all directions when occupied */
    for (int i = 0; i < NUM_ZONES; i++) {
        if (i != ZONE_CENTER) {
            safety_system->conflicting_moves[ZONE_CENTER][i] = true;
            safety_system->conflicting_moves[i][ZONE_CENTER] = true;
        }
    }

    /* Same direction movements don't conflict */
    safety_system->conflicting_moves[ZONE_NORTH_ENTRY][ZONE_SOUTH_ENTRY] = false;
    safety_system->conflicting_moves[ZONE_SOUTH_ENTRY][ZONE_NORTH_ENTRY] = false;
    safety_system->conflicting_moves[ZONE_WEST_ENTRY][ZONE_EAST_ENTRY] = false;
    safety_system->conflicting_moves[ZONE_EAST_ENTRY][ZONE_WEST_ENTRY] = false;
}

int compare_resource_priority(int zone1, int zone2)
{
    int priority_order[] = { ZONE_NORTH_ENTRY, ZONE_WEST_ENTRY, ZONE_CENTER, ZONE_EAST_ENTRY, ZONE_SOUTH_ENTRY };

    for (int i = 0; i < NUM_ZONES; i++) {
        if (priority_order[i] == zone1) return -1;
        if (priority_order[i] == zone2) return 1;
    }
    return 0;
}

void sort_zones_by_priority(int zones[], int num_zones)
{
    for (int i = 0; i < num_zones - 1; i++) {
        for (int j = 0; j < num_zones - i - 1; j++) {
            if (compare_resource_priority(zones[j], zones[j + 1]) > 0) {
                int temp = zones[j];
                zones[j] = zones[j + 1];
                zones[j + 1] = temp;
            }
        }
    }
}

bool handle_ambulance_priority(struct vehicle_info* vi, struct position next_pos)
{
    int zone = get_zone_for_position(next_pos);
    int priority = get_vehicle_priority(vi);

    int time_left = vi->golden_time - crossroads_step;
    if (time_left <= 1) {
        /* Emergency! Preempt all vehicles */
        preempt_normal_vehicles(vi);
        priority = PRIORITY_AMBULANCE + 3; /* Maximum priority */
    }

    /* Try to acquire resources with priority */
    priority_sema_down(&deadlock_system->intersection_capacity, priority);
    priority_lock_acquire(&deadlock_system->zone_locks[zone], priority);

    deadlock_system->zones_occupied[zone] = true;
    deadlock_system->zone_holders[zone] = vi->id;

    return true;
}

void preempt_normal_vehicles(struct vehicle_info* ambulance)
{
    printf("EMERGENCY: Ambulance %c preempting normal vehicles\n", ambulance->id);

    /* In a real implementation, we would signal vehicles to yield */
    /* For now, we rely on priority mechanisms */
    for (int i = 0; i < NUM_ZONES; i++) {
        if (deadlock_system->zones_occupied[i] &&
            deadlock_system->zone_holders[i] != ambulance->id) {
            printf("Zone %d should be evacuated for ambulance %c\n", i, ambulance->id);
        }
    }
}

