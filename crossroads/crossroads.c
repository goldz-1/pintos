#include <stdio.h>
#include <string.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/crossroads/crossroads.h"
#include "projects/crossroads/blinker.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"

#include "projects/crossroads/ats.h"

int crossroads_step;

static void init_map_locks(struct lock ***map_locks) 
{
	int i, j;
	struct lock **__map_locks;

	__map_locks = *map_locks = malloc(sizeof (struct lock *) * 7);
	for (i=0; i<7; i++) {
		__map_locks[i] = malloc(sizeof (struct lock) * 7);
		for (j=0; j<7; j++) {
			lock_init(&__map_locks[i][j]);
		}
	}
}

static void release_map_locks(struct lock **map_locks)
{
	int i;
	for (i=0; i<7; i++) {
		free(map_locks[i]);
	}
	free(map_locks);
}

static int is_finished(struct vehicle_info *vehicle_info, int thread_cnt)
{
	int i, res;

	res = thread_cnt;
	for (i=0; i<thread_cnt; i++) {
		if (vehicle_info[i].state == VEHICLE_STATUS_FINISHED) {
			res--;
		}
	}

	/* return 0 when terminated */
	return res;
}

void run_crossroads(char **argv)
{
	int i, thread_cnt;
	struct lock **map_locks;
	struct vehicle_info *vehicle_info;
	struct blinker_info* blinkers;

	/* initialize unit step */
	crossroads_step = 0;

	/* prepare crossroads map */
	init_map_locks(&map_locks);

	/* prepare vehicle data */
	thread_cnt = 1;
	for (i=0; (size_t) i<strlen(argv[1]); i++) {
		if (argv[1][i] == ':') {
			thread_cnt++;
		}
	}
	printf("initializing %d vehicles...\n", thread_cnt);
	vehicle_info = malloc(sizeof(struct vehicle_info) * thread_cnt);
	parse_vehicles(vehicle_info, argv[1]);

	for (i=0; i<thread_cnt; i++) {
		/* put map locks */
		vehicle_info[i].map_locks = map_locks;
	}

	init_on_mainthread(thread_cnt);

	blinkers = malloc(sizeof(struct blinker_info) * NUM_BLINKER);
	init_blinker(blinkers, map_locks, vehicle_info);

	/* prepare threads for each vehicle */ 
	printf("initializing vehicle threads...\n");
	for (i=0; i<thread_cnt; i++) {
		char name[16];
		snprintf(name, sizeof name, "thread %c", vehicle_info[i].id);
		thread_create(name, PRI_DEFAULT, vehicle_loop, &vehicle_info[i]);
	}
	start_blinker();

	printf("running project2 crossroads ...\n");

#if 1
	/* main loop */
	do {
		map_draw();
		for (i=0; i<thread_cnt; i++) {
			map_draw_vehicle(vehicle_info[i].id, 
							vehicle_info[i].position.row,
							vehicle_info[i].position.col);
		}
		/* sleep */
		timer_msleep(1000);
	} while (is_finished(vehicle_info, thread_cnt));

	/* dealloc */
	map_draw_reset();
	printf("finished. releasing resources ...\n");
	release_map_locks(map_locks);
	free(vehicle_info);
	free(blinkers);
	printf("good bye.\n");
#endif
}
