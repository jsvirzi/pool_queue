#include <strings.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "pool_queue.h"

void initialize_pool_queue(PoolQueue *pool, unsigned int buffs, unsigned int buff_size) {
    bzero(pool, sizeof(PoolQueue));
    pool->mask = PoolQueueBuffs - 1;
    pool->buff_mask = PoolQueueBuffLength - 1;
    pool->head = 0;
    pool->tail = 0;
#if 0
    uint8_t *big_buff;
    if (pool->buff) { big_buff = (uint8_t *) pool->buff; }
    else { big_buff = (uint8_t *) malloc(buffs * buff_size); }
    printf("big buff @ %p\n", (void *) big_buff);
    pool->buff = (uint8_t **) malloc(buffs * sizeof(uint8_t *));
    pool->mask = buffs - 1;
    pool->length = (unsigned int *) malloc(buffs * sizeof(unsigned int));
    for (unsigned int i = 0; i < buffs; ++i) {
        pool->buff[i] = (uint8_t *) &big_buff[i * buff_size];
        if (i < 128) { printf("pool pointer %d = %p. size = %d\n", i, (void *) pool->buff[i], buff_size); }
        pool->length[i] = 0;
    }
    pool->buff_mask = buff_size - 1;
#endif

}

void release_pool_queue(PoolQueue *pool) {
//    free(pool->buff[0]); /* points to big buffer */
//    free(pool->buff);
}

unsigned int stream_pool_queue(PoolQueue *pool, int fd, unsigned int max_bytes) {
    ssize_t remaining = max_bytes, n_read;
    unsigned int buff_size = pool->buff_mask + 1;
    while (remaining > pool->mask) {
        n_read = read(fd, pool->buff[pool->head], buff_size);
        if (n_read > 0) {
            remaining = remaining - n_read;
            unsigned int new_head = (pool->head + 1) & pool->mask;
            if (new_head != pool->tail) { pool->head = new_head; }
            else { return (unsigned int) (max_bytes - remaining); }
        }
    }
    if (remaining) {
        n_read = read(fd, pool->buff[pool->head], (size_t) remaining);
        if (n_read > 0) {
            pool->head = (pool->head + 1) & pool->mask;
            remaining = remaining - n_read;
            unsigned int new_head = (pool->head + 1) & pool->mask;
            if (new_head != pool->tail) { pool->head = new_head; }
            else { return (unsigned int) (max_bytes - remaining); }
        }
    }
    return (unsigned int) (max_bytes - remaining);
}
