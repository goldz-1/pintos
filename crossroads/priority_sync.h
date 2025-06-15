#ifndef __PROJECTS_CROSSROADS_PRIORITY_SYNC_H__
#define __PROJECTS_CROSSROADS_PRIORITY_SYNC_H__

#include <stdbool.h>
#include "threads/synch.h"
#include "lib/kernel/list.h"

/* Forward declarations */
struct vehicle_info;

/* Priority levels */
#define PRIORITY_AMBULANCE 3
#define PRIORITY_TRAFFIC_LIGHT 2  
#define PRIORITY_NORMAL_VEHICLE 1

/* Priority semaphore structure */
struct priority_sema {
    int value;              /* Semaphore value */
    struct list waiters;    /* List of waiting threads */
    struct lock lock;       /* Lock for internal use */
};

/* Priority lock structure */
struct priority_lock {
    struct priority_sema semaphore; /* Internal semaphore */
    struct thread *holder;           /* Current holder */
};

/* Priority condition variable */
struct priority_condition {
    struct list waiters;    /* List of waiting threads */
};

/* Waiter information structure */
struct priority_waiter {
    struct list_elem elem;      /* List element */
    struct thread *thread;      /* Waiting thread */
    int priority;               /* Thread priority */
    struct semaphore sema;      /* Private semaphore for signaling */
};

/* Priority semaphore functions */
void priority_sema_init(struct priority_sema *sema, int value);
void priority_sema_down(struct priority_sema *sema, int priority);
bool priority_sema_try_down(struct priority_sema *sema, int priority);
void priority_sema_up(struct priority_sema *sema);

/* Priority lock functions */
void priority_lock_init(struct priority_lock *lock);
void priority_lock_acquire(struct priority_lock *lock, int priority);
bool priority_lock_try_acquire(struct priority_lock *lock, int priority);
void priority_lock_release(struct priority_lock *lock);

/* Priority condition variable functions */
void priority_cond_init(struct priority_condition *cond);
void priority_cond_wait(struct priority_condition *cond, struct priority_lock *lock, int priority);
void priority_cond_signal(struct priority_condition *cond, struct priority_lock *lock);
void priority_cond_broadcast(struct priority_condition *cond, struct priority_lock *lock);

/* Utility functions */
int get_vehicle_priority(struct vehicle_info *vi);
bool priority_waiter_less(const struct list_elem *a, const struct list_elem *b, void *aux);

#endif /* __PROJECTS_CROSSROADS_PRIORITY_SYNC_H__ */