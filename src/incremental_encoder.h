#ifndef _DEFINE_INCREMENTAL_ENCODER_H
#define _DEFINE_INCREMENTAL_ENCODER_H

#include <stdint.h>

struct incremental_encoder
{
    uint32_t state;
};

extern int32_t get_coder_left(void);
extern int32_t get_coder_right(void);

extern void attach_encoder(void);

#endif // _DEFINE_INCREMENTAL_ENCODER_H
