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

void initialize_pool_queue(PoolQueue *pool, int buffs, int buff_size) {
    bzero(pool, sizeof(PoolQueue));
    uint8_t *big_buff = (uint8_t *) malloc(buffs * buff_size);
    pool->buff = (uint8_t **) malloc(buffs * sizeof(uint8_t *));
    pool->mask = buffs - 1;
    pool->length = (unsigned int *) malloc(buffs * sizeof(unsigned int));
    for (int i = 0; i < buffs; ++i) {
        pool->buff[i] = (uint8_t *) &big_buff[i * buff_size];
    }
    pool->buff_mask = buff_size - 1;
}

void release_pool_queue(PoolQueue *pool) {
    free(pool->buff[0]); /* points to big buffer */
    free(pool->buff);
}

int stream_pool_queue(PoolQueue *pool, int fd, int max_bytes) {
    int remaining = max_bytes, n_read;
    unsigned int buff_size = pool->buff_mask + 1;
    while (remaining > pool->mask) {
        n_read = read(fd, pool->buff[pool->head], buff_size);
        if (n_read > 0) {
            remaining = remaining - n_read;
            unsigned int new_head = (pool->head + 1) & pool->mask;
            if (new_head != pool->tail) { pool->head = new_head; }
            else { return max_bytes - remaining; }
        }
    }
    if (remaining) {
        n_read = read(fd, pool->buff[pool->head], remaining);
        if (n_read > 0) {
            pool->head = (pool->head + 1) & pool->mask;
            remaining = remaining - n_read;
            unsigned int new_head = (pool->head + 1) & pool->mask;
            if (new_head != pool->tail) { pool->head = new_head; }
            else { return max_bytes - remaining; }
        }
    }
    return (max_bytes - remaining);
}