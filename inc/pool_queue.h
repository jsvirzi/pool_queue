#ifndef POOLQUEUE_H
#define POOLQUEUE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct PoolQueue {
    uint8_t **buff;
    unsigned int *length;
    unsigned int head;
    unsigned int tail;
    unsigned int mask;
    unsigned int buff_mask;
} PoolQueue;

void initialize_pool_queue(PoolQueue *pool, int buffs, int buff_size);
void release_pool_queue(PoolQueue *pool);

#ifdef __cplusplus
}
#endif

#endif

