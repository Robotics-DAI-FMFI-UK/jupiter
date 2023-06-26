#ifndef _FIND_BRICK_H_
#define _FIND_BRICK_H_

#include <inttypes.h>


extern uint8_t *buffer;
extern int sirka, vyska;

void najdi_kocku(int *sirka_kocky, int *vyska_kocky, int *velkost_kocky, int *riadok, int *stlpec);

#endif
