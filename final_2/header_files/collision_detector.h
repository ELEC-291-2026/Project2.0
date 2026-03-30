#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include <stdint.h>

typedef struct
{
    uint16_t distance_mm;
    int initialized;
    int measurement_valid;
    int obstacle_detected;
} collision_detector_t;

void collision_detector_reset(collision_detector_t *detector);
int collision_detector_init(collision_detector_t *detector);
void collision_detector_update(collision_detector_t *detector);

#endif
