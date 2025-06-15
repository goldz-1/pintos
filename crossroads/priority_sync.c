#include "projects/crossroads/priority_sync.h"
#include "projects/crossroads/vehicle.h"
#include "threads/thread.h"
#include "threads/interrupt.h"
#include <stdio.h>

// compare priority 
bool priority_waiter_less(const struct list_elem *a, const struct list_elem *b, void *aux UNUSED)
{
    struct priority_waiter *wa = list_entry(a, struct priority_waiter, elem);
    struct priority_waiter *wb = list_entry(b, struct priority_waiter, elem);
    
    return wa->priority > wb->priority; 
}

// decide vehicle priority
int get_vehicle_priority(struct vehicle_info *vi)
{
    if (vi->type == VEHICL_TYPE_AMBULANCE) {
        // abulance: neer ro golden time 
        int time_left = vi->golden_time - crossroads_step;
        if (time_left <= 2) {
            return PRIORITY_AMBULANCE + 2; 
        } else if (time_left <= 5) {
            return PRIORITY_AMBULANCE + 1; 
        } else {
            return PRIORITY_AMBULANCE;     
        }
    }
    return PRIORITY_NORMAL_VEHICLE;
}


void priority_sema_init(struct priority_sema *sema, int value)
{
    ASSERT(sema != NULL);
    ASSERT(value >= 0);
    
    sema->value = value;
    list_init(&sema->waiters);
    lock_init(&sema->lock);
}

void priority_sema_down(struct priority_sema *sema, int priority)
{
    struct priority_waiter waiter;
    enum intr_level old_level;
    
    ASSERT(sema != NULL);
    ASSERT(!intr_context());
    
    // initialize waiter
    waiter.thread = thread_current();
    waiter.priority = priority;
    sema_init(&waiter.sema, 0);
    
    old_level = intr_disable();
    lock_acquire(&sema->lock);
    
    if (sema->value > 0) {
        sema->value--;
        lock_release(&sema->lock);
        intr_set_level(old_level);
        return;
    }
    
    list_insert_ordered(&sema->waiters, &waiter.elem, priority_waiter_less, NULL);
    
    lock_release(&sema->lock);
    intr_set_level(old_level);
    
    sema_down(&waiter.sema);
}

bool priority_sema_try_down(struct priority_sema *sema, int priority)
{
    enum intr_level old_level;
    bool success = false;
    
    ASSERT(sema != NULL);
    
    old_level = intr_disable();
    lock_acquire(&sema->lock);
    
    if (sema->value > 0) {
        sema->value--;
        success = true;
    }
    
    lock_release(&sema->lock);
    intr_set_level(old_level);
    
    return success;
}

void priority_sema_up(struct priority_sema *sema)
{
    struct priority_waiter *waiter;
    enum intr_level old_level;
    
    ASSERT(sema != NULL);
    
    old_level = intr_disable();
    lock_acquire(&sema->lock);
    
    if (!list_empty(&sema->waiters)) {
        waiter = list_entry(list_pop_front(&sema->waiters), struct priority_waiter, elem);
        sema_up(&waiter->sema);
    }
    else {
        sema->value++;
    }
    
    lock_release(&sema->lock);
    intr_set_level(old_level);
}


void priority_lock_init(struct priority_lock *lock)
{
    ASSERT(lock != NULL);
    
    priority_sema_init(&lock->semaphore, 1);
    lock->holder = NULL;
}

void priority_lock_acquire(struct priority_lock *lock, int priority)
{
    ASSERT(lock != NULL);
    ASSERT(!intr_context());
    ASSERT(lock->holder != thread_current());
    
    priority_sema_down(&lock->semaphore, priority);
    lock->holder = thread_current();
}

bool priority_lock_try_acquire(struct priority_lock *lock, int priority)
{
    ASSERT(lock != NULL);
    ASSERT(lock->holder != thread_current());
    
    bool success = priority_sema_try_down(&lock->semaphore, priority);
    if (success) {
        lock->holder = thread_current();
    }
    return success;
}

void priority_lock_release(struct priority_lock *lock)
{
    ASSERT(lock != NULL);
    ASSERT(lock->holder == thread_current());
    
    lock->holder = NULL;
    priority_sema_up(&lock->semaphore);
}


void priority_cond_init(struct priority_condition *cond)
{
    ASSERT(cond != NULL);
    list_init(&cond->waiters);
}

void priority_cond_wait(struct priority_condition *cond, struct priority_lock *lock, int priority)
{
    struct priority_waiter waiter;
    
    ASSERT(cond != NULL);
    ASSERT(lock != NULL);
    ASSERT(lock->holder == thread_current());
    
    waiter.thread = thread_current();
    waiter.priority = priority;
    sema_init(&waiter.sema, 0);
    
    list_insert_ordered(&cond->waiters, &waiter.elem, priority_waiter_less, NULL);
    
    priority_lock_release(lock);
    sema_down(&waiter.sema);
    priority_lock_acquire(lock, priority);
}

void priority_cond_signal(struct priority_condition *cond, struct priority_lock *lock)
{
    ASSERT(cond != NULL);
    ASSERT(lock != NULL);
    ASSERT(lock->holder == thread_current());
    
    if (!list_empty(&cond->waiters)) {
        struct priority_waiter *waiter = list_entry(list_pop_front(&cond->waiters), 
                                                   struct priority_waiter, elem);
        sema_up(&waiter->sema);
    }
}

void priority_cond_broadcast(struct priority_condition *cond, struct priority_lock *lock)
{
    ASSERT(cond != NULL);
    ASSERT(lock != NULL);
    ASSERT(lock->holder == thread_current());
    
    while (!list_empty(&cond->waiters)) {
        priority_cond_signal(cond, lock);
    }
}