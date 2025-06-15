#ifndef __PROJECTS_CROSSROADS_PRIORITY_SYNC_H__
#define __PROJECTS_CROSSROADS_PRIORITY_SYNC_H__

#include "threads/synch.h"
#include "lib/kernel/list.h"

// define priority 
#define PRIORITY_AMBULANCE 3
#define PRIORITY_TRAFFIC_LIGHT 2  
#define PRIORITY_NORMAL_VEHICLE 1

// priority semaphore 
struct priority_sema {
    int value;              // semaphore value 
    struct list waiters;    // waiting queue
    struct lock lock;       
};

// priority lock
struct priority_lock {
    struct priority_sema semaphore; 
    struct thread *holder;
};

struct priority_condition {
    struct list waiters;
};

// waiter info
struct priority_waiter {
    struct list_elem elem;     
    struct thread *thread;      
    int priority;               
    struct semaphore sema;
};

// priority semaphore functions
void priority_sema_init(struct priority_sema *sema, int value);
void priority_sema_down(struct priority_sema *sema, int priority);
bool priority_sema_try_down(struct priority_sema *sema, int priority);
void priority_sema_up(struct priority_sema *sema);

// priority semaphore lock functions
void priority_lock_init(struct priority_lock *lock);
void priority_lock_acquire(struct priority_lock *lock, int priority);
bool priority_lock_try_acquire(struct priority_lock *lock, int priority);
void priority_lock_release(struct priority_lock *lock);

//priority codition variable functions
void priority_cond_init(struct priority_condition *cond);
void priority_cond_wait(struct priority_condition *cond, struct priority_lock *lock, int priority);
void priority_cond_signal(struct priority_condition *cond, struct priority_lock *lock);
void priority_cond_broadcast(struct priority_condition *cond, struct priority_lock *lock);

//utility functions
int get_vehicle_priority(struct vehicle_info *vi);
bool priority_waiter_less(const struct list_elem *a, const struct list_elem *b, void *aux);

#endif /* __PROJECTS_CROSSROADS_PRIORITY_SYNC_H__ */