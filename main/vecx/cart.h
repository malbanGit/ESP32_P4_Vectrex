#ifndef CART_H
#define CART_H

#ifdef __cplusplus
extern "C" {
#endif

void cartInit(char *filename);
unsigned char cartRead(int addr);
void cartWrite(int addr, unsigned char data);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif