#ifndef __E8910_H
#define __E8910_H

#ifdef __cplusplus
extern "C" {
#endif

void e8910_init_sound();
void e8910_done_sound();
void e8910_write(int r, int v);
void e8910_tick();
void e8910_setdac(int val);
int e8910_read(int r);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

