#ifndef POOLQUEUE_H
#define POOLQUEUE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define PoolQueueBuffs (128)
#define PoolQueueBuffLength (1024)
typedef struct PoolQueue {
    // uint8_t **buff;
    uint8_t buff[PoolQueueBuffs][PoolQueueBuffLength];
    // unsigned int *length;
    unsigned int length[PoolQueueBuffs];
    unsigned int head;
    unsigned int tail;
    unsigned int mask;
    unsigned int buff_mask;
} PoolQueue;

void initialize_pool_queue(PoolQueue *pool, unsigned int buffs, unsigned int buff_size);
void release_pool_queue(PoolQueue *pool);

#ifdef __cplusplus
}
#endif

#endif

