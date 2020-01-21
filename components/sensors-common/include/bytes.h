
#ifndef _BYTES_H_INCLUDED_
#define _BYTES_H_INCLUDED_


#include <stdint.h>


static inline uint16_t
bytes_be_to_u16(unsigned char *data)
{
    return ((uint16_t) data[0] << 8) | (uint16_t) data[1];
}


#endif /* _BYTES_H_INCLUDED_ */
