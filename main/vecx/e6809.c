#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "esp_attr.h"

#include "e6809.h"

#define einline __inline

//extern void //vecx_intermediateSteps(int count);
//extern void //vecx_intermediateStepsUC(int count);



/* code assumptions:
 *  - it is assumed that an 'int' is at least 16 bits long.
 *  - a 16-bit register has valid bits only in the lower 16 bits and an
 *    8-bit register has valid bits only in the lower 8 bits. the upper
 *    may contain garbage!
 *  - all reading functions are assumed to return the requested data in
 *    the lower bits with the unused upper bits all set to zero.
 */

enum {
	FLAG_E		= 0x80,
	FLAG_F		= 0x40,
	FLAG_H		= 0x20,
	FLAG_I		= 0x10,
	FLAG_N		= 0x08,
	FLAG_Z		= 0x04,
	FLAG_V		= 0x02,
	FLAG_C		= 0x01,
	IRQ_NORMAL	= 0,
	IRQ_SYNC	= 1,
	IRQ_CWAI	= 2
};
int addressBUS = 0;
unsigned char dataBUS = 0;

static int PRE_CLR_STEPS = 4;
static int POST_CLR_ADDSTEPS = -1;

/* index registers */

DRAM_ATTR static unsigned reg_x;
DRAM_ATTR static unsigned reg_y;

/* user stack pointer */

DRAM_ATTR static unsigned reg_u;

/* hardware stack pointer */

DRAM_ATTR static unsigned reg_s;

/* program counter */

DRAM_ATTR unsigned reg_pc;

/* accumulators */

DRAM_ATTR static unsigned reg_a;
DRAM_ATTR static unsigned reg_b;

/* direct page register */

DRAM_ATTR static unsigned reg_dp;

/* condition codes */

DRAM_ATTR static unsigned reg_cc;

/* flag to see if interrupts should be handled (sync/cwai). */

DRAM_ATTR static unsigned irq_status;

DRAM_ATTR static unsigned *rptr_xyus[4] = {
	&reg_x,
	&reg_y,
	&reg_u,
	&reg_s
};


/* user defined read and write functions */

DRAM_ATTR unsigned char (*e6809_read8) (unsigned address);
DRAM_ATTR void (*e6809_write8) (unsigned address, unsigned char data);

/* obtain a particular condition code. returns 0 or 1. */

static einline unsigned get_cc (unsigned flag)
{
	return (reg_cc / flag) & 1;
}

/* set a particular condition code to either 0 or 1.
 * value parameter must be either 0 or 1.
 */

static einline void set_cc (unsigned flag, unsigned value)
{
	reg_cc = value?(reg_cc | flag):(reg_cc & ~flag);
}

/* test carry */

static einline unsigned test_c (unsigned i0, unsigned i1,
								unsigned r, unsigned sub)
{
	return (((((i0 | i1) & ~r) | (i0 & i1)) & 0x80) == 0x80) ^ (sub);
}

/* test negative */

static einline unsigned test_n (unsigned r)
{
	return (r & 0x80) == 0x80;
}

/* test for zero in lower 8 bits */

static einline unsigned test_z8 (unsigned r)
{
	return (r&0xff) == 0;
}

/* test for zero in lower 16 bits */

static einline unsigned test_z16 (unsigned r)
{
	return (r&0xffff) == 0;
}

/* overflow is set whenever the sign bits of the inputs are the same
 * but the sign bit of the result is not same as the sign bits of the
 * inputs.
 */

static einline unsigned test_v (unsigned i0, unsigned i1, unsigned r)
{
	return (((~(i0 ^ i1)) & (i0 ^ r)) & 0x80) == 0x80;
}

static einline unsigned get_reg_d (void)
{
	return (((reg_a << 8)&0xff00) | (reg_b & 0xff));
}

static einline void set_reg_d (unsigned value)
{
	reg_a = (value >> 8) & 0xff;
	reg_b = value & 0xff;
}

/* read a byte ... the returned value has the lower 8-bits set to the byte
 * while the upper bits are all zero.
 */

static einline unsigned read8 (unsigned address)
{
	return (*e6809_read8) (address & 0xffff)&0xff;
}

/* write a byte ... only the lower 8-bits of the unsigned data
 * is written. the upper bits are ignored.
 */

static einline void write8 (unsigned address, unsigned data)
{
	(*e6809_write8) (address & 0xffff, (unsigned char) data);
}

static einline unsigned read16 (unsigned address)
{
	unsigned datahi = read8 (address);
	unsigned datalo = read8 (address + 1);
	return (datahi << 8) | datalo;
}
static einline unsigned read16_cycloid (unsigned address)
{
	unsigned datahi = read8 (address);
//    //vecx_intermediateStepsUC(1);
	unsigned datalo = read8 (address + 1);
	return (datahi << 8) | datalo;
}

static einline void write16 (unsigned address, unsigned data)
{
	write8 (address, data >> 8);
	write8 (address + 1, data);
}
static einline void write16_cycloid (unsigned address, unsigned data)
{
	write8 (address, data >> 8);
//    //vecx_intermediateStepsUC(1);
	write8 (address + 1, data);
}

static einline void push8 (unsigned *sp, unsigned data)
{
	(*sp)--;
	write8 (*sp, data);
}

static einline unsigned pull8 (unsigned *sp)
{
	unsigned	data = read8 (*sp);
	(*sp)++;

	return data;
}

static einline void push16 (unsigned *sp, unsigned data)
{
	push8 (sp, data);
	push8 (sp, data >> 8);
}

static einline unsigned pull16 (unsigned *sp)
{
	unsigned datahi = pull8 (sp);
	unsigned datalo = pull8 (sp);

	return (datahi << 8) | datalo;
}

/* read a byte from the address pointed to by the pc */

static einline unsigned pc_read8 (void)
{
	unsigned data = read8 (reg_pc);
    reg_pc=(reg_pc+1)&0xffff;

	return data;
}

/* read a word from the address pointed to by the pc */

static einline unsigned pc_read16 (void)
{
	unsigned data = read16 (reg_pc);
    reg_pc=(reg_pc+2)&0xffff;

	return data;
}

/* sign extend an 8-bit quantity into a 16-bit quantity */

static einline unsigned sign_extend (unsigned data)
{
	return (~(data & 0x80) + 1) | (data & 0xff);
}

/* direct addressing, upper byte of the address comes from
 * the direct page register, and the lower byte comes from the
 * instruction itself.
 */

static einline unsigned ea_direct (void)
{
	return (reg_dp << 8) | pc_read8 ();
}

/* extended addressing, address is obtained from 2 bytes following
 * the instruction.
 */

static einline unsigned ea_extended (void)
{
	return addressBUS = pc_read16 ();
}

/* indexed addressing */

static einline unsigned ea_indexed (unsigned *cycles)
{
	unsigned r, op, ea;

	/* post byte */

	op = pc_read8 ();

	r = (op >> 5) & 3;

	switch (op) {
	case 0x00: case 0x01: case 0x02: case 0x03:
	case 0x04: case 0x05: case 0x06: case 0x07:
	case 0x08: case 0x09: case 0x0a: case 0x0b:
	case 0x0c: case 0x0d: case 0x0e: case 0x0f:
	case 0x20: case 0x21: case 0x22: case 0x23:
	case 0x24: case 0x25: case 0x26: case 0x27:
	case 0x28: case 0x29: case 0x2a: case 0x2b:
	case 0x2c: case 0x2d: case 0x2e: case 0x2f:
	case 0x40: case 0x41: case 0x42: case 0x43:
	case 0x44: case 0x45: case 0x46: case 0x47:
	case 0x48: case 0x49: case 0x4a: case 0x4b:
	case 0x4c: case 0x4d: case 0x4e: case 0x4f:
	case 0x60: case 0x61: case 0x62: case 0x63:
	case 0x64: case 0x65: case 0x66: case 0x67:
	case 0x68: case 0x69: case 0x6a: case 0x6b:
	case 0x6c: case 0x6d: case 0x6e: case 0x6f:
		/* R, +[0, 15] */
//		//vecx_intermediateSteps(1);
		ea = *rptr_xyus[r] + (op & 0xf);
		(*cycles)++;
		break;
	case 0x10: case 0x11: case 0x12: case 0x13:
	case 0x14: case 0x15: case 0x16: case 0x17:
	case 0x18: case 0x19: case 0x1a: case 0x1b:
	case 0x1c: case 0x1d: case 0x1e: case 0x1f:
	case 0x30: case 0x31: case 0x32: case 0x33:
	case 0x34: case 0x35: case 0x36: case 0x37:
	case 0x38: case 0x39: case 0x3a: case 0x3b:
	case 0x3c: case 0x3d: case 0x3e: case 0x3f:
	case 0x50: case 0x51: case 0x52: case 0x53:
	case 0x54: case 0x55: case 0x56: case 0x57:
	case 0x58: case 0x59: case 0x5a: case 0x5b:
	case 0x5c: case 0x5d: case 0x5e: case 0x5f:
	case 0x70: case 0x71: case 0x72: case 0x73:
	case 0x74: case 0x75: case 0x76: case 0x77:
	case 0x78: case 0x79: case 0x7a: case 0x7b:
	case 0x7c: case 0x7d: case 0x7e: case 0x7f:
		/* R, +[-16, -1] */
//		//vecx_intermediateSteps(1);

		ea = *rptr_xyus[r] + (op & 0xf) - 0x10;
		(*cycles)++;
		break;
	case 0x80: case 0x81:
	case 0xa0: case 0xa1:
	case 0xc0: case 0xc1:
	case 0xe0: case 0xe1:
		/* ,R+ / ,R++ */
//		//vecx_intermediateSteps(2 + (op & 1));

		ea = *rptr_xyus[r];
		*rptr_xyus[r] += 1 + (op & 1);
		*cycles += 2 + (op & 1);
		break;
	case 0x90: case 0x91:
	case 0xb0: case 0xb1:
	case 0xd0: case 0xd1:
	case 0xf0: case 0xf1:
		/* [,R+] ??? / [,R++] */

//		//vecx_intermediateSteps(5 + (op & 1));
		ea = read16 (*rptr_xyus[r]);
		*rptr_xyus[r] += 1 + (op & 1);
		*cycles += 5 + (op & 1);
		break;
	case 0x82: case 0x83:
	case 0xa2: case 0xa3:
	case 0xc2: case 0xc3:
	case 0xe2: case 0xe3:

		/* ,-R / ,--R */

//		//vecx_intermediateSteps(2 + (op & 1));
		*rptr_xyus[r] -= 1 + (op & 1);
		ea = *rptr_xyus[r];
		*cycles += 2 + (op & 1);
		break;
	case 0x92: case 0x93:
	case 0xb2: case 0xb3:
	case 0xd2: case 0xd3:
	case 0xf2: case 0xf3:
		/* [,-R] ??? / [,--R] */

//		//vecx_intermediateSteps(5 + (op & 1));
		*rptr_xyus[r] -= 1 + (op & 1);
		ea = read16 (*rptr_xyus[r]);
		*cycles += 5 + (op & 1);
		break;
	case 0x84: case 0xa4:
	case 0xc4: case 0xe4:
		/* ,R */

		ea = *rptr_xyus[r];
		break;
	case 0x94: case 0xb4:
	case 0xd4: case 0xf4:
		/* [,R] */

//		//vecx_intermediateSteps(3);
		ea = read16 (*rptr_xyus[r]);
		*cycles += 3;
		break;
	case 0x85: case 0xa5:
	case 0xc5: case 0xe5:
		/* B,R */

//		//vecx_intermediateSteps(1);
		ea = *rptr_xyus[r] + sign_extend (reg_b);
		*cycles += 1;
		break;
	case 0x95: case 0xb5:
	case 0xd5: case 0xf5:
		/* [B,R] */

//		//vecx_intermediateSteps(4);
		ea = read16 (*rptr_xyus[r] + sign_extend (reg_b));
		*cycles += 4;
		break;
	case 0x86: case 0xa6:
	case 0xc6: case 0xe6:
		/* A,R */

		////vecx_intermediateSteps(1);
		ea = *rptr_xyus[r] + sign_extend (reg_a);
		*cycles += 1;
		break;
	case 0x96: case 0xb6:
	case 0xd6: case 0xf6:
		/* [A,R] */

//		//vecx_intermediateSteps(4);
		ea = read16 (*rptr_xyus[r] + sign_extend (reg_a));
		*cycles += 4;
		break;
	case 0x88: case 0xa8:
	case 0xc8: case 0xe8:
		/* byte,R */

//		//vecx_intermediateSteps(1);
		ea = *rptr_xyus[r] + sign_extend (pc_read8 ());
		*cycles += 1;
		break;
	case 0x98: case 0xb8:
	case 0xd8: case 0xf8:
		/* [byte,R] */

//		//vecx_intermediateSteps(4);
		ea = read16 (*rptr_xyus[r] + sign_extend (pc_read8 ()));
		*cycles += 4;
		break;
	case 0x89: case 0xa9:
	case 0xc9: case 0xe9:
		/* word,R */

//		//vecx_intermediateSteps(4);
		ea = *rptr_xyus[r] + pc_read16 ();
		*cycles += 4;
		break;
	case 0x99: case 0xb9:
	case 0xd9: case 0xf9:
		/* [word,R] */

		//vecx_intermediateSteps(7);
		ea = read16 (*rptr_xyus[r] + pc_read16 ());
		*cycles += 7;
		break;
	case 0x8b: case 0xab:
	case 0xcb: case 0xeb:
		/* D,R */

		//vecx_intermediateSteps(4);
		ea = *rptr_xyus[r] + get_reg_d ();
		*cycles += 4;
		break;
	case 0x9b: case 0xbb:
	case 0xdb: case 0xfb:
		/* [D,R] */

		//vecx_intermediateSteps(7);
		ea = read16 (*rptr_xyus[r] + get_reg_d ());
		*cycles += 7;
		break;
	case 0x8c: case 0xac:
	case 0xcc: case 0xec:
		/* byte, PC */

		//vecx_intermediateSteps(1);
		r = sign_extend (pc_read8 ());
		ea = reg_pc + r;
		*cycles += 1;
		break;
	case 0x9c: case 0xbc:
	case 0xdc: case 0xfc:
		/* [byte, PC] */

		//vecx_intermediateSteps(4);
		r = sign_extend (pc_read8 ());
		ea = read16 (reg_pc + r);
		*cycles += 4;
		break;
	case 0x8d: case 0xad:
	case 0xcd: case 0xed:
		/* word, PC */

		//vecx_intermediateSteps(5);
		r = pc_read16 ();
		ea = reg_pc + r;
		*cycles += 5;
		break;
	case 0x9d: case 0xbd:
	case 0xdd: case 0xfd:
		/* [word, PC] */

		//vecx_intermediateSteps(8);
		r = pc_read16 ();
		ea = read16 (reg_pc + r);
		*cycles += 8;
		break;
	case 0x9f:
		/* [address] */

		//vecx_intermediateSteps(5);
		ea = read16 (pc_read16 ());
		*cycles += 5;
		break;
	default:
		break;
	}

	return ea;
}

/* instruction: neg
 * essentially (0 - data).
 */

static einline unsigned inst_neg (unsigned data)
{
	unsigned i0 = 0;
	unsigned i1 = ~data;
	unsigned r = i0 + i1 + 1;

    //set_cc (FLAG_H, test_c (i0 << 4, i1 << 4, r << 4, 0));
    reg_cc = test_c(i0 << 4, i1 << 4, r << 4, 0)?(reg_cc | FLAG_H):(reg_cc & ~FLAG_H);
    
    //set_cc (FLAG_N, test_n (r));
    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    
    //set_cc (FLAG_Z, test_z8 (r));
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
   
    //set_cc (FLAG_V, test_v (i0, i1, r));
    reg_cc = test_v(i0, i1, r)?(reg_cc | FLAG_V):(reg_cc & ~FLAG_V);

    //set_cc (FLAG_C, test_c (i0, i1, r, 1));
    reg_cc = test_c(i0, i1, r, 1)?(reg_cc | FLAG_C):(reg_cc & ~FLAG_C);

	return r;
}

/* instruction: com */

static einline unsigned inst_com (unsigned data)
{
	unsigned r = ~data;

    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);

    reg_cc = (reg_cc & ~FLAG_V);
    reg_cc = (reg_cc | FLAG_C);

	return r;
}

/* instruction: lsr
 * cannot be faked as an add or substract.
 */

static einline unsigned inst_lsr (unsigned data)
{
	unsigned r = (data >> 1) & 0x7f;

    //set_cc (FLAG_N, 0);
    reg_cc = (reg_cc & ~FLAG_N);
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    //set_cc (FLAG_C, (data & 1)==1 );
    reg_cc = ((data & 1)==1)?(reg_cc | FLAG_C):(reg_cc & ~FLAG_C);

	return r;
}

/* instruction: ror
 * cannot be faked as an add or substract.
 */

static einline unsigned inst_ror (unsigned data)
{
	unsigned     r = ((data >> 1) & 0x7f) | ((((reg_cc & FLAG_C) == FLAG_C)?1:0) << 7);

    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = ((data & 1)==1)?(reg_cc | FLAG_C):(reg_cc & ~FLAG_C);

	return r;
} 

/* instruction: asr
 * cannot be faked as an add or substract.
 */

static einline unsigned inst_asr (unsigned data)
{
	unsigned     r = ((data >> 1) & 0x7f) | (data & 0x80);

    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = ((data & 1)==1)?(reg_cc | FLAG_C):(reg_cc & ~FLAG_C);

	return r;
}

/* instruction: asl
 * essentially (data + data). simple addition.
 */

static einline unsigned inst_asl (unsigned data)
{
	unsigned i0 = data;
	unsigned i1 = data;
	unsigned r = i0 + i1;

    reg_cc = test_c(i0 << 4, i1 << 4, r << 4, 0)?(reg_cc | FLAG_H):(reg_cc & ~FLAG_H);
    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = (((data & 0x40)>>6)!=((data & 0x80)>>7))?(reg_cc | FLAG_V):(reg_cc & ~FLAG_V);
    reg_cc = ((data&0x80)==0x80)?(reg_cc | FLAG_C):(reg_cc & ~FLAG_C);
    
	return r&0xff;
}

/* instruction: rol
 * essentially (data + data + carry). addition with carry.
 */

static einline unsigned inst_rol (unsigned data)
{
	unsigned r = data + data + (((reg_cc & FLAG_C) == FLAG_C)?1:0);

    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = (((data & 0x40)>>6)!=((data & 0x80)>>7))?(reg_cc | FLAG_V):(reg_cc & ~FLAG_V);
    reg_cc = ((data&0x80)==0x80)?(reg_cc | FLAG_C):(reg_cc & ~FLAG_C);
    

	return r&0xff;;
}

/* instruction: dec
 * essentially (data - 1).
 */

static einline unsigned inst_dec (unsigned data)
{
	unsigned     r = (data + 0xff)&0xff;

    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = (r== 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = (data==0x80)?(reg_cc | FLAG_V):(reg_cc & ~FLAG_V);


	return r;
}

/* instruction: inc
 * essentially (data + 1).
 */

static einline unsigned inst_inc (unsigned data)
{
	unsigned r = (data + 1)&0xff;

    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = (r == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = (data==0x7f)?(reg_cc | FLAG_V):(reg_cc & ~FLAG_V);


	return r;
}

/* instruction: tst */

static einline void inst_tst8 (unsigned data)
{
    reg_cc = ((data & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((data&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = (reg_cc & ~FLAG_V);
}

static einline void inst_tst16 (unsigned data)
{
    reg_cc =  (((data) & 0x8000) == 0x8000) ?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((data&0xffff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = (reg_cc & ~FLAG_V);
}

/* instruction: clr */

static einline void inst_clr (void)
{
    reg_cc = (reg_cc & ~FLAG_N);
    reg_cc = (reg_cc | FLAG_Z);
    reg_cc = (reg_cc & ~FLAG_V);
    reg_cc = (reg_cc & ~FLAG_C);
}

/* instruction: suba/subb */

static einline unsigned inst_sub8 (unsigned data0, unsigned data1)
{
	unsigned i0 = data0;
	unsigned i1 = ~data1;
	unsigned r = i0 + i1 + 1;

    reg_cc = test_c(i0 << 4, i1 << 4, r << 4, 0)?(reg_cc | FLAG_H):(reg_cc & ~FLAG_H);
    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = test_v(i0, i1, r)?(reg_cc | FLAG_V):(reg_cc & ~FLAG_V);
    reg_cc = test_c(i0, i1, r, 1)?(reg_cc | FLAG_C):(reg_cc & ~FLAG_C);

	return r&0xff;
}

/* instruction: sbca/sbcb/cmpa/cmpb.
 * only 8-bit version, 16-bit version not needed.
 */

static einline unsigned inst_sbc (unsigned data0, unsigned data1)
{
	unsigned i0;
	unsigned i1;
	unsigned r;

    i0 = data0;
    i1 = ~data1;
    //c = 1 - get_cc_int (FLAG_C);

    r = i0 + i1 + (((reg_cc & FLAG_C) == FLAG_C)?0:1);

    reg_cc = test_c(i0 << 4, i1 << 4, r << 4, 0)?(reg_cc | FLAG_H):(reg_cc & ~FLAG_H);
    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = test_v(i0, i1, r)?(reg_cc | FLAG_V):(reg_cc & ~FLAG_V);
    reg_cc = test_c(i0, i1, r, 1)?(reg_cc | FLAG_C):(reg_cc & ~FLAG_C);

    return r&0xff;
}

/* instruction: anda/andb/bita/bitb.
 * only 8-bit version, 16-bit version not needed.
 */

static einline unsigned inst_and (unsigned data0, unsigned data1)
{
	unsigned r ;


    r = data0 & data1;
    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = (reg_cc & ~FLAG_V);

	return r;
}

/* instruction: eora/eorb.
 * only 8-bit version, 16-bit version not needed.
 */

static einline unsigned inst_eor (unsigned data0, unsigned data1)
{
	unsigned     r = data0 ^ data1;
    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = (reg_cc & ~FLAG_V);


	return r;
}

/* instruction: adca/adcb
 * only 8-bit version, 16-bit version not needed.
 */

static einline unsigned inst_adc (unsigned data0, unsigned data1)
{
	unsigned i0;
	unsigned i1;
	unsigned r;

    i0 = data0;
    i1 = data1;
    r = (i0 + i1 + (((reg_cc & FLAG_C) == FLAG_C)?1:0))&0xff;

    reg_cc = test_c(i0 << 4, i1 << 4, r << 4, 0)?(reg_cc | FLAG_H):(reg_cc & ~FLAG_H);
    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = test_v(i0, i1, r)?(reg_cc | FLAG_V):(reg_cc & ~FLAG_V);
    reg_cc = test_c (i0, i1, r, 0)?(reg_cc | FLAG_C):(reg_cc & ~FLAG_C);


	return r;
}

/* instruction: ora/orb.
 * only 8-bit version, 16-bit version not needed.
 */

static einline unsigned inst_or (unsigned data0, unsigned data1)
{
	unsigned     r = data0 | data1;

    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = (reg_cc & ~FLAG_V);

	return r;
}

/* instruction: adda/addb */

static einline unsigned inst_add8 (unsigned data0, unsigned data1)
{
	unsigned i0;
	unsigned i1;
	unsigned r;

    i0 = data0;
    i1 = data1;
    r = (i0 + i1)&0xff;

    reg_cc = test_c(i0 << 4, i1 << 4, r << 4, 0)?(reg_cc | FLAG_H):(reg_cc & ~FLAG_H);
    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = (r == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = test_v(i0, i1, r)?(reg_cc | FLAG_V):(reg_cc & ~FLAG_V);
    reg_cc = test_c (i0, i1, r, 0)?(reg_cc | FLAG_C):(reg_cc & ~FLAG_C);
	return r;
}

/* instruction: addd */

static einline unsigned inst_add16 (unsigned data0, unsigned data1)
{
	unsigned i0;
	unsigned i1;
	unsigned r;

    i0 = data0;
    i1 = data1;
    r = (i0 + i1)&0xffff;

    reg_cc = ((r & 0x8000) == 0x8000)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = (r == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = test_v (i0 >> 8, i1 >> 8, r >> 8)?(reg_cc | FLAG_V):(reg_cc & ~FLAG_V);
    reg_cc = test_c (i0 >> 8, i1 >> 8, r >> 8, 0)?(reg_cc | FLAG_C):(reg_cc & ~FLAG_C);
    

	return r;
}

/* instruction: subd */

static einline unsigned inst_sub16 (unsigned data0, unsigned data1)
{
	unsigned i0;
	unsigned i1;
	unsigned r;

    i0 = data0;
    i1 = ~data1;
    r = (i0 + i1 + 1)&0xffff;

    reg_cc = ((r & 0x8000) == 0x8000)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = (r == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = test_v (i0 >> 8, i1 >> 8, r >> 8)?(reg_cc | FLAG_V):(reg_cc & ~FLAG_V);
    reg_cc = test_c (i0 >> 8, i1 >> 8, r >> 8, 1)?(reg_cc | FLAG_C):(reg_cc & ~FLAG_C);


	return r;
}

/* instruction: 8-bit offset branch */

static einline void inst_bra8 (unsigned test, unsigned op, unsigned *cycles)
{
    int offset;
    offset = pc_read8 ();
    if (!((test) ^ ((op&1)==1)))
    {
        reg_pc = (reg_pc + ((~((offset) & 0x80) + 1) | ((offset) & 0xff) ))& 0xffff;;
    }

	*cycles += 3;
}

/* instruction: 16-bit offset branch */

static einline void inst_bra16 (unsigned test, unsigned op, unsigned *cycles)
{
	unsigned offset = pc_read16 ();

    if (!((test) ^ ((op&1)==1)))
    {
        reg_pc = (reg_pc + offset)& 0xffff;
        //vecx_intermediateSteps(1);
        *cycles += 1;
    }
    *cycles += 5;
    //vecx_intermediateSteps(5);	
}

/* instruction: pshs/pshu */

static einline void inst_psh (unsigned op, unsigned *sp,
					   unsigned data, unsigned *cycles)
{
	if (op & 0x80) {
		push16 (sp, reg_pc);
		*cycles += 2;
		//vecx_intermediateSteps(2);
	}

	if (op & 0x40) {
		/* either s or u */
		push16 (sp, data);
		*cycles += 2;
		//vecx_intermediateSteps(2);
	}

	if (op & 0x20) {
		push16 (sp, reg_y);
		*cycles += 2;
		//vecx_intermediateSteps(2);
	}

	if (op & 0x10) {
		push16 (sp, reg_x);
		*cycles += 2;
		//vecx_intermediateSteps(2);
	}

	if (op & 0x08) {
		push8 (sp, reg_dp);
		*cycles += 1;
		//vecx_intermediateSteps(1);
	}

	if (op & 0x04) {
		push8 (sp, reg_b);
		*cycles += 1;
		//vecx_intermediateSteps(1);
	}

	if (op & 0x02) {
		push8 (sp, reg_a);
		*cycles += 1;
		//vecx_intermediateSteps(1);
	}

	if (op & 0x01) {
		push8 (sp, reg_cc);
		*cycles += 1;
		//vecx_intermediateSteps(1);
	}
}

/* instruction: puls/pulu */

static einline void inst_pul (unsigned op, unsigned *sp, unsigned *osp,
					   unsigned *cycles)
{
	if (op & 0x01) {
		reg_cc = pull8 (sp);
		*cycles += 1;
		//vecx_intermediateSteps(1);
	}

	if (op & 0x02) {
		reg_a = pull8 (sp);
		*cycles += 1;
		//vecx_intermediateSteps(1);
	}

	if (op & 0x04) {
		reg_b = pull8 (sp);
		*cycles += 1;
		//vecx_intermediateSteps(1);
	}

	if (op & 0x08) {
		reg_dp = pull8 (sp);
		*cycles += 1;
		//vecx_intermediateSteps(1);
	}

	if (op & 0x10) {
		reg_x = pull16 (sp);
		*cycles += 2;
		//vecx_intermediateSteps(2);
	}

	if (op & 0x20) {
		reg_y = pull16 (sp);
		*cycles += 2;
		//vecx_intermediateSteps(2);
	}

	if (op & 0x40) {
		/* either s or u */
		*osp = pull16 (sp);
		*cycles += 2;
		//vecx_intermediateSteps(2);
	}

	if (op & 0x80) {
		reg_pc = pull16 (sp);
		*cycles += 2;
		//vecx_intermediateSteps(2);
	}
}

static einline unsigned exgtfr_read (unsigned reg)
{
   unsigned data;

   switch (reg)
   {
      case 0x0:
         data = (((reg_a << 8)&0xff00) | (reg_b & 0xff));
         break;
      case 0x1:
         data = reg_x;
         break;
      case 0x2:
         data = reg_y;
         break;
      case 0x3:
         data = reg_u;
         break;
      case 0x4:
         data = reg_s;
         break;
      case 0x5:
         data = reg_pc;
         break;
      case 0x8:
         data = 0xff00 | reg_a;
         break;
      case 0x9:
         data = 0xff00 | reg_b;
         break;
      case 0xa:
         data = 0xff00 | reg_cc;
         break;
      case 0xb:
         data = 0xff00 | reg_dp;
         break;
      default:
         data = 0xffff;
         break;
   }

   return data;
}

static einline void exgtfr_write (unsigned reg, unsigned data)
{
   switch (reg)
   {
      case 0x0:
         set_reg_d (data);
         break;
      case 0x1:
         reg_x = data;
         break;
      case 0x2:
         reg_y = data;
         break;
      case 0x3:
         reg_u = data;
         break;
      case 0x4:
         reg_s = data;
         break;
      case 0x5:
         reg_pc = data;
         break;
      case 0x8:
         reg_a = data&0xff;
         break;
      case 0x9:
         reg_b = data&0xff;
         break;
      case 0xa:
         reg_cc = data&0xff;
         break;
      case 0xb:
         reg_dp = data&0xff;
         break;
      default:
         break;
   }
}

/* instruction: exg */

static einline void inst_exg (void)
{
	unsigned op, tmp;

	op = pc_read8 ();

	tmp = exgtfr_read (op & 0xf);
	exgtfr_write (op & 0xf, exgtfr_read ((op >> 4) &0xf));
	exgtfr_write (op >> 4, tmp);
}


/* instruction: tfr */

static einline void inst_tfr (void)
{
	unsigned op;

	op = pc_read8 ();

	exgtfr_write (op & 0xf, exgtfr_read ((op >> 4) &0xf));
}

/* reset the 6809 */

void e6809_reset (void)
{
	reg_x = 0;
	reg_y = 0;
	reg_u = 0;
	reg_s = 0;

	reg_a = 0;
	reg_b = 0;

	reg_dp = 0;

	reg_cc = FLAG_I | FLAG_F;
	irq_status = IRQ_NORMAL;

	reg_pc = read16 (0xfffe);
}
/* execute a single instruction or handle interrupts and return */


IRAM_ATTR unsigned e6809_sstep (unsigned irq_i, unsigned irq_f)
{
    unsigned op;
    unsigned cycles = 0;
    unsigned ea, i0, i1, r;

    if (irq_f) 
    {
        if (((reg_cc & FLAG_F) != FLAG_F)) 
        {
            if (irq_status != IRQ_CWAI) 
            {
                reg_cc = (reg_cc & ~FLAG_E);
                inst_psh (0x81, &reg_s, reg_u, &cycles);
            }

            reg_cc = (reg_cc | FLAG_I);
            reg_cc = (reg_cc | FLAG_F);

            reg_pc = read16 (0xfff6);
            irq_status = IRQ_NORMAL;
            cycles += 7;
        } 
        else 
        {
            if (irq_status == IRQ_SYNC) 
            {
                irq_status = IRQ_NORMAL;
            }
        }
    }

    if (irq_i) 
    {
        if (((reg_cc & FLAG_I) != FLAG_I)) 
        {
            if (irq_status != IRQ_CWAI) 
            {
                reg_cc = (reg_cc | FLAG_E);
                inst_psh (0xff, &reg_s, reg_u, &cycles);
            }

            reg_cc = (reg_cc | FLAG_I);

            reg_pc = read16 (0xfff8);
            irq_status = IRQ_NORMAL;
            cycles += 7;
        } 
        else 
        {
            if (irq_status == IRQ_SYNC) 
            {
                irq_status = IRQ_NORMAL;
            }
        }
    }

    if (irq_status != IRQ_NORMAL) {
        return cycles + 1;
    }

    op = pc_read8 ();

    /* computed-goto Dispatch-Tabelle */
    static void *opcode_table[256] = {
        [0 ... 255] = &&op_default,

        [0x00] = &&op_00,
        [0x03] = &&op_03,
        [0x04] = &&op_04,
        [0x06] = &&op_06,
        [0x07] = &&op_07,
        [0x08] = &&op_08,
        [0x09] = &&op_09,
        [0x0a] = &&op_0a,
        [0x0c] = &&op_0c,
        [0x0d] = &&op_0d,
        [0x0e] = &&op_0e,
        [0x0f] = &&op_0f,

        [0x10] = &&op_10,
        [0x11] = &&op_11,
        [0x12] = &&op_12,
        [0x13] = &&op_13,
        [0x16] = &&op_16,
        [0x17] = &&op_17,
        [0x19] = &&op_19,
        [0x1a] = &&op_1a,
        [0x1c] = &&op_1c,
        [0x1d] = &&op_1d,
        [0x1e] = &&op_1e,
        [0x1f] = &&op_1f,

        [0x20] = &&op_20,
        [0x21] = &&op_21,
        [0x22] = &&op_22,
        [0x23] = &&op_23,
        [0x24] = &&op_24,
        [0x25] = &&op_25,
        [0x26] = &&op_26,
        [0x27] = &&op_27,
        [0x28] = &&op_28,
        [0x29] = &&op_29,
        [0x2a] = &&op_2a,
        [0x2b] = &&op_2b,
        [0x2c] = &&op_2c,
        [0x2d] = &&op_2d,
        [0x2e] = &&op_2e,
        [0x2f] = &&op_2f,

        [0x30] = &&op_30,
        [0x31] = &&op_31,
        [0x32] = &&op_32,
        [0x33] = &&op_33,
        [0x34] = &&op_34,
        [0x35] = &&op_35,
        [0x36] = &&op_36,
        [0x37] = &&op_37,

        [0x39] = &&op_39,
        [0x3a] = &&op_3a,
        [0x3b] = &&op_3b,
        [0x3c] = &&op_3c,
        [0x3d] = &&op_3d,
        [0x3f] = &&op_3f,

        [0x40] = &&op_40,
        [0x43] = &&op_43,
        [0x44] = &&op_44,
        [0x46] = &&op_46,
        [0x47] = &&op_47,
        [0x48] = &&op_48,
        [0x49] = &&op_49,
        [0x4a] = &&op_4a,
        [0x4c] = &&op_4c,
        [0x4d] = &&op_4d,
        [0x4f] = &&op_4f,

        [0x50] = &&op_50,
        [0x53] = &&op_53,
        [0x54] = &&op_54,
        [0x56] = &&op_56,
        [0x57] = &&op_57,
        [0x58] = &&op_58,
        [0x59] = &&op_59,
        [0x5a] = &&op_5a,
        [0x5c] = &&op_5c,
        [0x5d] = &&op_5d,
        [0x5f] = &&op_5f,

        [0x60] = &&op_60,
        [0x63] = &&op_63,
        [0x64] = &&op_64,
        [0x66] = &&op_66,
        [0x67] = &&op_67,
        [0x68] = &&op_68,
        [0x69] = &&op_69,
        [0x6a] = &&op_6a,
        [0x6c] = &&op_6c,
        [0x6d] = &&op_6d,
        [0x6e] = &&op_6e,
        [0x6f] = &&op_6f,

        [0x70] = &&op_70,
        [0x73] = &&op_73,
        [0x74] = &&op_74,
        [0x76] = &&op_76,
        [0x77] = &&op_77,
        [0x78] = &&op_78,
        [0x79] = &&op_79,
        [0x7a] = &&op_7a,
        [0x7c] = &&op_7c,
        [0x7d] = &&op_7d,
        [0x7e] = &&op_7e,
        [0x7f] = &&op_7f,

        [0x80] = &&op_80,
        [0x81] = &&op_81,
        [0x82] = &&op_82,
        [0x83] = &&op_83,
        [0x84] = &&op_84,
        [0x85] = &&op_85,
        [0x86] = &&op_86,
        [0x88] = &&op_88,
        [0x89] = &&op_89,
        [0x8a] = &&op_8a,
        [0x8b] = &&op_8b,
        [0x8c] = &&op_8c,
        [0x8d] = &&op_8d,
        [0x8e] = &&op_8e,

        [0x90] = &&op_90,
        [0x91] = &&op_91,
        [0x92] = &&op_92,
        [0x93] = &&op_93,
        [0x94] = &&op_94,
        [0x95] = &&op_95,
        [0x96] = &&op_96,
        [0x97] = &&op_97,
        [0x98] = &&op_98,
        [0x99] = &&op_99,
        [0x9a] = &&op_9a,
        [0x9b] = &&op_9b,
        [0x9c] = &&op_9c,
        [0x9d] = &&op_9d,
        [0x9e] = &&op_9e,
        [0x9f] = &&op_9f,

        [0xa0] = &&op_a0,
        [0xa1] = &&op_a1,
        [0xa2] = &&op_a2,
        [0xa3] = &&op_a3,
        [0xa4] = &&op_a4,
        [0xa5] = &&op_a5,
        [0xa6] = &&op_a6,
        [0xa7] = &&op_a7,
        [0xa8] = &&op_a8,
        [0xa9] = &&op_a9,
        [0xaa] = &&op_aa,
        [0xab] = &&op_ab,
        [0xac] = &&op_ac,
        [0xad] = &&op_ad,
        [0xae] = &&op_ae,
        [0xaf] = &&op_af,

        [0xb0] = &&op_b0,
        [0xb1] = &&op_b1,
        [0xb2] = &&op_b2,
        [0xb3] = &&op_b3,
        [0xb4] = &&op_b4,
        [0xb5] = &&op_b5,
        [0xb6] = &&op_b6,
        [0xb7] = &&op_b7,
        [0xb8] = &&op_b8,
        [0xb9] = &&op_b9,
        [0xba] = &&op_ba,
        [0xbb] = &&op_bb,
        [0xbc] = &&op_bc,
        [0xbd] = &&op_bd,
        [0xbe] = &&op_be,
        [0xbf] = &&op_bf,

        [0xc0] = &&op_c0,
        [0xc1] = &&op_c1,
        [0xc2] = &&op_c2,
        [0xc3] = &&op_c3,
        [0xc4] = &&op_c4,
        [0xc5] = &&op_c5,
        [0xc6] = &&op_c6,
        [0xc8] = &&op_c8,
        [0xc9] = &&op_c9,
        [0xca] = &&op_ca,
        [0xcb] = &&op_cb,
        [0xcc] = &&op_cc,
        [0xce] = &&op_ce,

        [0xd0] = &&op_d0,
        [0xd1] = &&op_d1,
        [0xd2] = &&op_d2,
        [0xd3] = &&op_d3,
        [0xd4] = &&op_d4,
        [0xd5] = &&op_d5,
        [0xd6] = &&op_d6,
        [0xd7] = &&op_d7,
        [0xd8] = &&op_d8,
        [0xd9] = &&op_d9,
        [0xda] = &&op_da,
        [0xdb] = &&op_db,
        [0xdc] = &&op_dc,
        [0xdd] = &&op_dd,
        [0xde] = &&op_de,
        [0xdf] = &&op_df,

        [0xe0] = &&op_e0,
        [0xe1] = &&op_e1,
        [0xe2] = &&op_e2,
        [0xe3] = &&op_e3,
        [0xe4] = &&op_e4,
        [0xe5] = &&op_e5,
        [0xe6] = &&op_e6,
        [0xe7] = &&op_e7,
        [0xe8] = &&op_e8,
        [0xe9] = &&op_e9,
        [0xea] = &&op_ea,
        [0xeb] = &&op_eb,
        [0xec] = &&op_ec,
        [0xed] = &&op_ed,
        [0xee] = &&op_ee,
        [0xef] = &&op_ef,

        [0xf0] = &&op_f0,
        [0xf1] = &&op_f1,
        [0xf2] = &&op_f2,
        [0xf3] = &&op_f3,
        [0xf4] = &&op_f4,
        [0xf5] = &&op_f5,
        [0xf6] = &&op_f6,
        [0xf7] = &&op_f7,
        [0xf8] = &&op_f8,
        [0xf9] = &&op_f9,
        [0xfa] = &&op_fa,
        [0xfb] = &&op_fb,
        [0xfc] = &&op_fc,
        [0xfd] = &&op_fd,
        [0xfe] = &&op_fe,
        [0xff] = &&op_ff,
    };

    goto *opcode_table[op];

/* ---------------- page 0 instructions ---------------- */

op_00: /* 0x00: neg direct */
    ea = ea_direct ();
    r = inst_neg (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_03: /* 0x03: com direct */
    ea = ea_direct ();
    r = inst_com (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_04: /* 0x04: lsr direct */
    ea = ea_direct ();
    r = inst_lsr (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_06: /* 0x06: ror direct */
    ea = ea_direct ();
    r = inst_ror (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_07: /* 0x07: asr direct */
    ea = ea_direct ();
    r = inst_asr (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_08: /* 0x08: asl direct */
    ea = ea_direct ();
    r = inst_asl (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_09: /* 0x09: rol direct */
    ea = ea_direct ();
    r = inst_rol (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_0a: /* 0x0a: dec direct */
    ea = ea_direct ();
    r = inst_dec (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_0c: /* 0x0c: inc direct */
    ea = ea_direct ();
    r = inst_inc (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_0d: /* 0x0d: tst direct */
    ea = ea_direct ();
    inst_tst8 (read8 (ea));
    cycles += 6;
    goto end;

op_0e: /* 0x0e: jmp direct */
    reg_pc = ea_direct ();
    cycles += 3;
    goto end;

op_0f: /* 0x0f: clr direct */
    ea = ea_direct ();
    inst_clr ();
    read8(ea); // clear reads! important for shift reg emulation!
    write8 (ea, 0);
    cycles += 6;
    goto end;

/* page 1 prefix */
op_10: /* 0x10 */
    op = pc_read8 ();
    switch (op) 
    {
        /* lbra / lbrn */
        case 0x20:
        case 0x21:
            inst_bra16 (0, op, &cycles);
            break;

        /* lbhi / lbls */
        case 0x22:
        case 0x23:
            inst_bra16 (get_cc (FLAG_C) | get_cc (FLAG_Z), op, &cycles);
            break;

        /* lbhs/lbcc / lblo/lbcs */
        case 0x24:
        case 0x25:
            inst_bra16 (get_cc (FLAG_C), op, &cycles);
            break;

        /* lbne / lbeq */
        case 0x26:
        case 0x27:
            inst_bra16 (get_cc (FLAG_Z), op, &cycles);
            break;

        /* lbvc / lbvs */
        case 0x28:
        case 0x29:
            inst_bra16 (get_cc (FLAG_V), op, &cycles);
            break;

        /* lbpl / lbmi */
        case 0x2a:
        case 0x2b:
            inst_bra16 (get_cc (FLAG_N), op, &cycles);
            break;

        /* lbge / lblt */
        case 0x2c:
        case 0x2d:
            inst_bra16 (get_cc (FLAG_N) ^ get_cc (FLAG_V), op, &cycles);
            break;

        /* lbgt / lble */
        case 0x2e:
        case 0x2f:
            inst_bra16 (get_cc (FLAG_Z) |
                        (get_cc (FLAG_N) ^ get_cc (FLAG_V)), op, &cycles);
            break;

        /* cmpd */
        case 0x83:
            inst_sub16 (get_reg_d (), pc_read16 ());
            cycles += 5;
            break;
        case 0x93:
            ea = ea_direct ();
            inst_sub16 (get_reg_d (), read16_cycloid (ea));
            cycles += 7;
            break;
        case 0xa3:
            ea = ea_indexed (&cycles);
            inst_sub16 (get_reg_d (), read16_cycloid (ea));
            cycles += 7;
            break;
        case 0xb3:
            ea = ea_extended ();
            inst_sub16 (get_reg_d (), read16_cycloid (ea));
            cycles += 8;
            break;

        /* cmpy */
        case 0x8c:
            inst_sub16 (reg_y, pc_read16 ());
            cycles += 5;
            break;
        case 0x9c:
            ea = ea_direct ();
            inst_sub16 (reg_y, read16 (ea));
            cycles += 7;
            break;
        case 0xac:
            ea = ea_indexed (&cycles);
            inst_sub16 (reg_y, read16 (ea));
            cycles += 7;
            break;
        case 0xbc:
            ea = ea_extended ();
            inst_sub16 (reg_y, read16 (ea));
            cycles += 8;
            break;

        /* ldy */
        case 0x8e:
            reg_y = pc_read16 ();
            inst_tst16 (reg_y);
            cycles += 4;
            break;
        case 0x9e:
            ea = ea_direct ();
            reg_y = read16 (ea);
            inst_tst16 (reg_y);
            cycles += 6;
            break;
        case 0xae:
            ea = ea_indexed (&cycles);
            reg_y = read16 (ea);
            inst_tst16 (reg_y);
            cycles += 6;
            break;
        case 0xbe:
            ea = ea_extended ();
            reg_y = read16 (ea);
            inst_tst16 (reg_y);
            cycles += 7;
            break;

        /* sty */
        case 0x9f:
            ea = ea_direct ();
            write16 (ea, reg_y);
            inst_tst16 (reg_y);
            cycles += 6;
            break;
        case 0xaf:
            ea = ea_indexed (&cycles);
            write16 (ea, reg_y);
            inst_tst16 (reg_y);
            cycles += 6;
            break;
        case 0xbf:
            ea = ea_extended ();
            write16 (ea, reg_y);
            inst_tst16 (reg_y);
            cycles += 7;
            break;

        /* lds */
        case 0xce:
            reg_s = pc_read16 ();
            inst_tst16 (reg_s);
            cycles += 4;
            break;
        case 0xde:
            ea = ea_direct ();
            reg_s = read16 (ea);
            inst_tst16 (reg_s);
            cycles += 6;
            break;
        case 0xee:
            ea = ea_indexed (&cycles);
            reg_s = read16 (ea);
            inst_tst16 (reg_s);
            cycles += 6;
            break;
        case 0xfe:
            ea = ea_extended ();
            reg_s = read16 (ea);
            inst_tst16 (reg_s);
            cycles += 7;
            break;

        /* sts */
        case 0xdf:
            ea = ea_direct ();
            write16 (ea, reg_s);
            inst_tst16 (reg_s);
            cycles += 6;
            break;
        case 0xef:
            ea = ea_indexed (&cycles);
            write16 (ea, reg_s);
            inst_tst16 (reg_s);
            cycles += 6;
            break;
        case 0xff:
            ea = ea_extended ();
            write16 (ea, reg_s);
            inst_tst16 (reg_s);
            cycles += 7;
            break;

        /* swi2 */
        case 0x3f:
            set_cc (FLAG_E, 1);
            inst_psh (0xff, &reg_s, reg_u, &cycles);
            reg_pc = read16 (0xfff4);
            cycles += 8;
            break;

        default:
            break;
    }
    goto end;

/* page 2 prefix */
op_11: /* 0x11 */
    op = pc_read8 ();

    switch (op) {
        /* cmpu */
        case 0x83:
            inst_sub16 (reg_u, pc_read16 ());
            cycles += 5;
            break;
        case 0x93:
            ea = ea_direct ();
            inst_sub16 (reg_u, read16 (ea));
            cycles += 7;
            break;
        case 0xa3:
            ea = ea_indexed (&cycles);
            inst_sub16 (reg_u, read16 (ea));
            cycles += 7;
            break;
        case 0xb3:
            ea = ea_extended ();
            inst_sub16 (reg_u, read16 (ea));
            cycles += 8;
            break;

        /* cmps */
        case 0x8c:
            inst_sub16 (reg_s, pc_read16 ());
            cycles += 5;
            break;
        case 0x9c:
            ea = ea_direct ();
            inst_sub16 (reg_s, read16 (ea));
            cycles += 7;
            break;
        case 0xac:
            ea = ea_indexed (&cycles);
            inst_sub16 (reg_s, read16 (ea));
            cycles += 7;
            break;
        case 0xbc:
            ea = ea_extended ();
            inst_sub16 (reg_s, read16 (ea));
            cycles += 8;
            break;

        /* swi3 */
        case 0x3f:
            set_cc (FLAG_E, 1);
            inst_psh (0xff, &reg_s, reg_u, &cycles);
            reg_pc = read16 (0xfff2);
            cycles += 8;
            break;

        default:
            break;
    }
    goto end;

/* nop */
op_12: /* 0x12 */
    cycles += 2;
    goto end;

/* sync */
op_13: /* 0x13 */
    irq_status = IRQ_SYNC;
    cycles += 2;
    goto end;

/* lbra */
op_16: /* 0x16 */
    r = pc_read16 ();
    reg_pc = (reg_pc + r) & 0xffff;
    cycles += 5;
    goto end;

/* lbsr */
op_17: /* 0x17 */
    r = pc_read16 ();
    push16 (&reg_s, reg_pc);
    reg_pc = (reg_pc + r) & 0xffff;
    cycles += 9;
    goto end;

/* daa */
op_19: /* 0x19 */
    i0 = reg_a;
    i1 = 0;

    if ((reg_a & 0x0f) > 0x09 || ((reg_cc & FLAG_H) == FLAG_H)) {
        i1 |= 0x06;
    }

    if ((reg_a & 0xf0) > 0x80 && (reg_a & 0x0f) > 0x09) {
        i1 |= 0x60;
    }

    if ((reg_a & 0xf0) > 0x90 || ((reg_cc & FLAG_C) == FLAG_C)) {
        i1 |= 0x60;
    }

    reg_a = (i0 + i1) & 0xff;

    set_cc (FLAG_N, test_n (reg_a));
    set_cc (FLAG_Z, test_z8 (reg_a));
    reg_cc = (reg_cc & ~FLAG_V);
    set_cc (FLAG_C, test_c (i0, i1, reg_a, 0));
    cycles += 2;
    goto end;

/* orcc */
op_1a: /* 0x1a */
    reg_cc |= pc_read8 ();
    cycles += 3;
    goto end;

/* andcc */
op_1c: /* 0x1c */
    reg_cc &= pc_read8 ();
    cycles += 3;
    goto end;

/* sex */
op_1d: /* 0x1d */
    set_reg_d (sign_extend (reg_b));
    set_cc (FLAG_N, test_n (reg_a));
    set_cc (FLAG_Z, test_z16 (get_reg_d ()));
    cycles += 2;
    goto end;

/* exg */
op_1e: /* 0x1e */
    inst_exg ();
    cycles += 8;
    goto end;

/* tfr */
op_1f: /* 0x1f */
    inst_tfr ();
    cycles += 6;
    goto end;

/* branches 0x20–0x2f */

op_20: /* 0x20 bra */
op_21: /* 0x21 brn */
    inst_bra8 (0, op, &cycles);
    goto end;

op_22: /* 0x22 bhi */
op_23: /* 0x23 bls */
    inst_bra8 (((reg_cc & FLAG_C) == FLAG_C) | ((reg_cc & FLAG_Z) == FLAG_Z), op, &cycles);
    goto end;

op_24: /* 0x24 bhs/bcc */
op_25: /* 0x25 blo/bcs */
    inst_bra8 (((reg_cc & FLAG_C) == FLAG_C), op, &cycles);
    goto end;

op_26: /* 0x26 bne */
op_27: /* 0x27 beq */
    inst_bra8 (((reg_cc & FLAG_Z) == FLAG_Z), op, &cycles);
    goto end;

op_28: /* 0x28 bvc */
op_29: /* 0x29 bvs */
    inst_bra8 (((reg_cc & FLAG_V) == FLAG_V), op, &cycles);
    goto end;

op_2a: /* 0x2a bpl */
op_2b: /* 0x2b bmi */
    inst_bra8 (((reg_cc & FLAG_N) == FLAG_N), op, &cycles);
    goto end;

op_2c: /* 0x2c bge */
op_2d: /* 0x2d blt */
    inst_bra8 (((reg_cc & FLAG_N) == FLAG_N) ^ ((reg_cc & FLAG_V) == FLAG_V), op, &cycles);
    goto end;

op_2e: /* 0x2e bgt */
op_2f: /* 0x2f ble */
    inst_bra8 (((reg_cc & FLAG_Z) == FLAG_Z) | (((reg_cc & FLAG_N) == FLAG_N) ^ ((reg_cc & FLAG_V) == FLAG_V)), op, &cycles);
    goto end;

/* leax/leay/leas/leau */

op_30: /* 0x30 leax */
    reg_x = ea_indexed (&cycles);
    set_cc (FLAG_Z, test_z16 (reg_x));
    cycles += 4;
    goto end;

op_31: /* 0x31 leay */
    reg_y = ea_indexed (&cycles);
    set_cc (FLAG_Z, test_z16 (reg_y));
    cycles += 4;
    goto end;

op_32: /* 0x32 leas */
    reg_s = ea_indexed (&cycles);
    cycles += 4;
    goto end;

op_33: /* 0x33 leau */
    reg_u = ea_indexed (&cycles);
    cycles += 4;
    goto end;

/* pshs/puls/pshu/pulu */

op_34: /* 0x34 pshs */
    inst_psh (pc_read8 (), &reg_s, reg_u, &cycles);
    cycles += 5;
    goto end;

op_35: /* 0x35 puls */
    inst_pul (pc_read8 (), &reg_s, &reg_u, &cycles);
    cycles += 5;
    goto end;

op_36: /* 0x36 pshu */
    inst_psh (pc_read8 (), &reg_u, reg_s, &cycles);
    cycles += 5;
    goto end;

op_37: /* 0x37 pulu */
    inst_pul (pc_read8 (), &reg_u, &reg_s, &cycles);
    cycles += 5;
    goto end;

/* rts */
op_39: /* 0x39 */
    reg_pc = pull16 (&reg_s);
    cycles += 5;
    goto end;

/* abx */
op_3a: /* 0x3a */
    reg_x += reg_b & 0xff;
    cycles += 3;
    goto end;

/* rti */
op_3b: /* 0x3b */
    inst_pul (0x01, &reg_s, &reg_u, &cycles);
    if (((reg_cc & FLAG_E) == FLAG_E))
    {
        inst_pul (0xfe, &reg_s, &reg_u, &cycles);
    } 
    else 
    {
        inst_pul (0x80, &reg_s, &reg_u, &cycles);
    }
    cycles += 3;
    goto end;

/* cwai */
op_3c: /* 0x3c */
    reg_cc &= pc_read8 ();
    set_cc (FLAG_E, 1);
    inst_psh (0xff, &reg_s, reg_u, &cycles);
    irq_status = IRQ_CWAI;
    cycles += 4;
    goto end;

/* mul */
op_3d: /* 0x3d */
    r = (reg_a & 0xff) * (reg_b & 0xff);
    set_reg_d (r);

    set_cc (FLAG_Z, test_z16 (r));
    set_cc (FLAG_C, (r >> 7) & 1);

    cycles += 11;
    goto end;

/* swi */
op_3f: /* 0x3f */
    set_cc (FLAG_E, 1);
    inst_psh (0xff, &reg_s, reg_u, &cycles);
    set_cc (FLAG_I, 1);
    set_cc (FLAG_F, 1);
    reg_pc = read16 (0xfffa);
    cycles += 7;
    goto end;

/* accumulator / memory ops for A/B, etc. */

/* neg / nega / negb */

op_40: /* 0x40 nega */
    reg_a = inst_neg (reg_a) & 0xff;
    cycles += 2;
    goto end;

op_43: /* 0x43 coma */
    reg_a = inst_com (reg_a);
    cycles += 2;
    goto end;

op_44: /* 0x44 lsra */
    reg_a = inst_lsr (reg_a);
    cycles += 2;
    goto end;

op_46: /* 0x46 rora */
    reg_a = inst_ror (reg_a);
    cycles += 2;
    goto end;

op_47: /* 0x47 asra */
    reg_a = inst_asr (reg_a);
    cycles += 2;
    goto end;

op_48: /* 0x48 asla */
    reg_a = inst_asl (reg_a);
    cycles += 2;
    goto end;

op_49: /* 0x49 rola */
    reg_a = inst_rol (reg_a);
    cycles += 2;
    goto end;

op_4a: /* 0x4a deca */
    reg_a = inst_dec (reg_a);
    cycles += 2;
    goto end;

op_4c: /* 0x4c inca */
    reg_a = inst_inc (reg_a);
    cycles += 2;
    goto end;

op_4d: /* 0x4d tsta */
    inst_tst8 (reg_a);
    cycles += 2;
    goto end;

op_4f: /* 0x4f clra */
    inst_clr ();
    reg_a = 0;
    cycles += 2;
    goto end;

/* B accumulator versions */

op_50: /* 0x50 negb */
    reg_b = inst_neg (reg_b) & 0xff;
    cycles += 2;
    goto end;

op_53: /* 0x53 comb */
    reg_b = inst_com (reg_b);
    cycles += 2;
    goto end;

op_54: /* 0x54 lsrb */
    reg_b = inst_lsr (reg_b);
    cycles += 2;
    goto end;

op_56: /* 0x56 rorb */
    reg_b = inst_ror (reg_b);
    cycles += 2;
    goto end;

op_57: /* 0x57 asrb */
    reg_b = inst_asr (reg_b);
    cycles += 2;
    goto end;

op_58: /* 0x58 aslb */
    reg_b = inst_asl (reg_b);
    cycles += 2;
    goto end;

op_59: /* 0x59 rolb */
    reg_b = inst_rol (reg_b);
    cycles += 2;
    goto end;

op_5a: /* 0x5a decb */
    reg_b = inst_dec (reg_b);
    cycles += 2;
    goto end;

op_5c: /* 0x5c incb */
    reg_b = inst_inc (reg_b);
    cycles += 2;
    goto end;

op_5d: /* 0x5d tstb */
    inst_tst8 (reg_b);
    cycles += 2;
    goto end;

op_5f: /* 0x5f clrb */
    inst_clr ();
    reg_b = 0;
    cycles += 2;
    goto end;

/* indexed/direct/extended versions for many ops... */

/* neg indexed */
op_60: /* 0x60 */
    ea = ea_indexed (&cycles);
    r = inst_neg (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_63: /* 0x63 com indexed */
    ea = ea_indexed (&cycles);
    r = inst_com (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_64: /* 0x64 lsr indexed */
    ea = ea_indexed (&cycles);
    r = inst_lsr (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_66: /* 0x66 ror indexed */
    ea = ea_indexed (&cycles);
    r = inst_ror (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_67: /* 0x67 asr indexed */
    ea = ea_indexed (&cycles);
    r = inst_asr (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_68: /* 0x68 asl indexed */
    ea = ea_indexed (&cycles);
    r = inst_asl (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_69: /* 0x69 rol indexed */
    ea = ea_indexed (&cycles);
    r = inst_rol (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_6a: /* 0x6a dec indexed */
    ea = ea_indexed (&cycles);
    r = inst_dec (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_6c: /* 0x6c inc indexed */
    ea = ea_indexed (&cycles);
    r = inst_inc (read8 (ea));
    write8 (ea, r);
    cycles += 6;
    goto end;

op_6d: /* 0x6d tst indexed */
    ea = ea_indexed (&cycles);
    inst_tst8 (read8 (ea));
    cycles += 6;
    goto end;

op_6e: /* 0x6e jmp indexed */
    reg_pc = ea_indexed (&cycles);
    cycles += 3;
    goto end;

op_6f: /* 0x6f clr indexed */
    ea = ea_indexed (&cycles);
    inst_clr ();
    read8(ea);
    write8 (ea, 0);
    cycles += 6;
    goto end;

/* extended: 0x70–0x7f */

op_70: /* 0x70 neg extended */
    ea = ea_extended ();
    r = inst_neg (read8 (ea));
    write8 (ea, r);
    cycles += 7;
    goto end;

op_73: /* 0x73 com extended */
    ea = ea_extended ();
    r = inst_com (read8 (ea));
    write8 (ea, r);
    cycles += 7;
    goto end;

op_74: /* 0x74 lsr extended */
    ea = ea_extended ();
    r = inst_lsr (read8 (ea));
    write8 (ea, r);
    cycles += 7;
    goto end;

op_76: /* 0x76 ror extended */
    ea = ea_extended ();
    r = inst_ror (read8 (ea));
    write8 (ea, r);
    cycles += 7;
    goto end;

op_77: /* 0x77 asr extended */
    ea = ea_extended ();
    r = inst_asr (read8 (ea));
    write8 (ea, r);
    cycles += 7;
    goto end;

op_78: /* 0x78 asl extended */
    ea = ea_extended ();
    r = inst_asl (read8 (ea));
    write8 (ea, r);
    cycles += 7;
    goto end;

op_79: /* 0x79 rol extended */
    ea = ea_extended ();
    r = inst_rol (read8 (ea));
    write8 (ea, r);
    cycles += 7;
    goto end;

op_7a: /* 0x7a dec extended */
    ea = ea_extended ();
    r = inst_dec (read8 (ea));
    write8 (ea, r);
    cycles += 7;
    goto end;

op_7c: /* 0x7c inc extended */
    ea = ea_extended ();
    r = inst_inc (read8 (ea));
    write8 (ea, r);
    cycles += 7;
    goto end;

op_7d: /* 0x7d tst extended */
    ea = ea_extended ();
    inst_tst8 (read8 (ea));
    cycles += 7;
    goto end;

op_7e: /* 0x7e jmp extended */
    reg_pc = ea_extended ();
    cycles += 4;
    goto end;

op_7f: /* 0x7f clr extended */
    ea = ea_extended ();
    inst_clr ();
    read8(ea);
    write8 (ea, 0);
    cycles += 7;
    goto end;

/* suba */
op_80: /* 0x80 */
    reg_a = inst_sub8 (reg_a, pc_read8 ()) & 0xff;
    cycles += 2;
    goto end;

op_90: /* 0x90 */
    ea = ea_direct ();
    reg_a = inst_sub8 (reg_a, read8 (ea)) & 0xff;
    cycles += 4;
    goto end;

op_a0: /* 0xa0 */
    ea = ea_indexed (&cycles);
    reg_a = inst_sub8 (reg_a, read8 (ea)) & 0xff;
    cycles += 4;
    goto end;

op_b0: /* 0xb0 */
    ea = ea_extended ();
    reg_a = inst_sub8 (reg_a, read8 (ea)) & 0xff;
    cycles += 5;
    goto end;

/* cmpa */
op_81: /* 0x81 */
    inst_sub8 (reg_a, pc_read8 ());
    cycles += 2;
    goto end;

op_91: /* 0x91 */
    ea = ea_direct ();
    inst_sub8 (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_a1: /* 0xa1 */
    ea = ea_indexed (&cycles);
    inst_sub8 (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_b1: /* 0xb1 */
    ea = ea_extended ();
    inst_sub8 (reg_a, read8 (ea));
    cycles += 5;
    goto end;

/* sbca */
op_82: /* 0x82 */
    reg_a = inst_sbc (reg_a, pc_read8 ());
    cycles += 2;
    goto end;

op_92: /* 0x92 */
    ea = ea_direct ();
    reg_a = inst_sbc (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_a2: /* 0xa2 */
    ea = ea_indexed (&cycles);
    reg_a = inst_sbc (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_b2: /* 0xb2 */
    ea = ea_extended ();
    reg_a = inst_sbc (reg_a, read8 (ea));
    cycles += 5;
    goto end;

/* subd */
op_83: /* 0x83 */
    set_reg_d (inst_sub16 (get_reg_d (), pc_read16 ()));
    cycles += 4;
    goto end;

op_93: /* 0x93 */
    ea = ea_direct ();
    set_reg_d (inst_sub16 (get_reg_d (), read16 (ea)));
    cycles += 6;
    goto end;

op_a3: /* 0xa3 */
    ea = ea_indexed (&cycles);
    set_reg_d (inst_sub16 (get_reg_d (), read16 (ea)));
    cycles += 6;
    goto end;

op_b3: /* 0xb3 */
    ea = ea_extended ();
    set_reg_d (inst_sub16 (get_reg_d (), read16 (ea)));
    cycles += 7;
    goto end;

/* anda */
op_84: /* 0x84 */
    reg_a = inst_and (reg_a, pc_read8 ());
    cycles += 2;
    goto end;

op_94: /* 0x94 */
    ea = ea_direct ();
    reg_a = inst_and (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_a4: /* 0xa4 */
    ea = ea_indexed (&cycles);
    reg_a = inst_and (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_b4: /* 0xb4 */
    ea = ea_extended ();
    reg_a = inst_and (reg_a, read8 (ea));
    cycles += 5;
    goto end;

/* bita */
op_85: /* 0x85 */
    inst_and (reg_a, pc_read8 ());
    cycles += 2;
    goto end;

op_95: /* 0x95 */
    ea = ea_direct ();
    inst_and (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_a5: /* 0xa5 */
    ea = ea_indexed (&cycles);
    inst_and (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_b5: /* 0xb5 */
    ea = ea_extended ();
    inst_and (reg_a, read8 (ea));
    cycles += 5;
    goto end;

/* lda */
op_86: /* 0x86 */
    dataBUS = reg_a = pc_read8 ();
    inst_tst8 (reg_a);
    cycles += 2;
    goto end;

op_96: /* 0x96 */
    ea = ea_direct ();
    reg_a = read8 (ea);
    inst_tst8 (reg_a);
    cycles += 4;
    goto end;

op_a6: /* 0xa6 */
    ea = ea_indexed (&cycles);
    reg_a = read8 (ea);
    inst_tst8 (reg_a);
    cycles += 4;
    goto end;

op_b6: /* 0xb6 */
    ea = ea_extended ();
    reg_a = read8 (ea);
    inst_tst8 (reg_a);
    cycles += 5;
    goto end;

/* bsr */
op_8d: /* 0x8d */
    r = pc_read8 ();
    push16 (&reg_s, reg_pc);
    reg_pc = (reg_pc + sign_extend (r)) & 0xffff;
    cycles += 7;
    goto end;

/* cmpx */
op_8c: /* 0x8c */
    inst_sub16 (reg_x, pc_read16 ());
    cycles += 4;
    goto end;

op_9c: /* 0x9c */
    ea = ea_direct ();
    inst_sub16 (reg_x, read16 (ea));
    cycles += 6;
    goto end;

op_ac: /* 0xac */
    ea = ea_indexed (&cycles);
    inst_sub16 (reg_x, read16 (ea));
    cycles += 6;
    goto end;

op_bc: /* 0xbc */
    ea = ea_extended ();
    inst_sub16 (reg_x, read16 (ea));
    cycles += 7;
    goto end;

/* ldx */
op_8e: /* 0x8e */
    reg_x = pc_read16 ();
    inst_tst16 (reg_x);
    cycles += 3;
    goto end;

op_9e: /* 0x9e */
    ea = ea_direct ();
    reg_x = read16 (ea);
    inst_tst16 (reg_x);
    cycles += 5;
    goto end;

op_ae: /* 0xae */
    ea = ea_indexed (&cycles);
    reg_x = read16 (ea);
    inst_tst16 (reg_x);
    cycles += 5;
    goto end;

op_be: /* 0xbe */
    ea = ea_extended ();
    reg_x = read16 (ea);
    inst_tst16 (reg_x);
    cycles += 6;
    goto end;

/* jsr */
op_9d: /* 0x9d */
    ea = ea_direct ();
    push16 (&reg_s, reg_pc);
    reg_pc = ea;
    cycles += 7;
    goto end;

op_ad: /* 0xad */
    ea = ea_indexed (&cycles);
    push16 (&reg_s, reg_pc);
    reg_pc = ea;
    cycles += 7;
    goto end;

op_bd: /* 0xbd */
    ea = ea_extended ();
    push16 (&reg_s, reg_pc);
    reg_pc = ea;
    cycles += 8;
    goto end;

/* sta */
op_97: /* 0x97 */
    ea = ea_direct ();
    write8 (ea, reg_a);
    inst_tst8 (reg_a);
    cycles += 4;
    goto end;

op_a7: /* 0xa7 */
    ea = ea_indexed (&cycles);
    write8 (ea, reg_a);
    inst_tst8 (reg_a);
    cycles += 4;
    goto end;

op_b7: /* 0xb7 */
    ea = ea_extended ();
    write8 (ea, reg_a);
    inst_tst8 (reg_a);
    cycles += 5;
    goto end;

/* stb */
op_d7: /* 0xd7 */
    ea = ea_direct ();
    write8 (ea, reg_b);
    inst_tst8 (reg_b);
    cycles += 4;
    goto end;

op_e7: /* 0xe7 */
    ea = ea_indexed (&cycles);
    write8 (ea, reg_b);
    inst_tst8 (reg_b);
    cycles += 4;
    goto end;

op_f7: /* 0xf7 */
    ea = ea_extended ();
    write8 (ea, reg_b);
    inst_tst8 (reg_b);
    cycles += 5;
    goto end;

/* eora */
op_88: /* 0x88 */
    reg_a = inst_eor (reg_a, pc_read8 ());
    cycles += 2;
    goto end;

op_98: /* 0x98 */
    ea = ea_direct ();
    reg_a = inst_eor (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_a8: /* 0xa8 */
    ea = ea_indexed (&cycles);
    reg_a = inst_eor (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_b8: /* 0xb8 */
    ea = ea_extended ();
    reg_a = inst_eor (reg_a, read8 (ea));
    cycles += 5;
    goto end;

/* adca */
op_89: /* 0x89 */
    reg_a = inst_adc (reg_a, pc_read8 ());
    cycles += 2;
    goto end;

op_99: /* 0x99 */
    ea = ea_direct ();
    reg_a = inst_adc (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_a9: /* 0xa9 */
    ea = ea_indexed (&cycles);
    reg_a = inst_adc (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_b9: /* 0xb9 */
    ea = ea_extended ();
    reg_a = inst_adc (reg_a, read8 (ea));
    cycles += 5;
    goto end;

/* ora */
op_8a: /* 0x8a */
    reg_a = inst_or (reg_a, pc_read8 ());
    cycles += 2;
    goto end;

op_9a: /* 0x9a */
    ea = ea_direct ();
    reg_a = inst_or (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_aa: /* 0xaa */
    ea = ea_indexed (&cycles);
    reg_a = inst_or (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_ba: /* 0xba */
    ea = ea_extended ();
    reg_a = inst_or (reg_a, read8 (ea));
    cycles += 5;
    goto end;

/* adda */
op_8b: /* 0x8b */
    reg_a = inst_add8 (reg_a, pc_read8 ());
    cycles += 2;
    goto end;

op_9b: /* 0x9b */
    ea = ea_direct ();
    reg_a = inst_add8 (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_ab: /* 0xab */
    ea = ea_indexed (&cycles);
    reg_a = inst_add8 (reg_a, read8 (ea));
    cycles += 4;
    goto end;

op_bb: /* 0xbb */
    ea = ea_extended ();
    reg_a = inst_add8 (reg_a, read8 (ea));
    cycles += 5;
    goto end;

/* ldd */
op_cc: /* 0xcc */
    set_reg_d (pc_read16 ());
    inst_tst16 (get_reg_d ());
    cycles += 3;
    goto end;

op_dc: /* 0xdc */
    ea = ea_direct ();
    set_reg_d (read16_cycloid (ea));
    inst_tst16 (get_reg_d ());
    cycles += 5;
    goto end;

op_ec: /* 0xec */
    ea = ea_indexed (&cycles);
    set_reg_d (read16_cycloid (ea));
    inst_tst16 (get_reg_d ());
    cycles += 5;
    goto end;

op_fc: /* 0xfc */
    ea = ea_extended ();
    set_reg_d (read16_cycloid (ea));
    inst_tst16 (get_reg_d ());
    cycles += 6;
    goto end;

/* cmpb */
op_c1: /* 0xc1 */
    inst_sub8 (reg_b, pc_read8 ());
    cycles += 2;
    goto end;

op_d1: /* 0xd1 */
    ea = ea_direct ();
    inst_sub8 (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_e1: /* 0xe1 */
    ea = ea_indexed (&cycles);
    inst_sub8 (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_f1: /* 0xf1 */
    ea = ea_extended ();
    inst_sub8 (reg_b, read8 (ea));
    cycles += 5;
    goto end;

/* sbcb */
op_c2: /* 0xc2 */
    reg_b = inst_sbc (reg_b, pc_read8 ());
    cycles += 2;
    goto end;

op_d2: /* 0xd2 */
    ea = ea_direct ();
    reg_b = inst_sbc (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_e2: /* 0xe2 */
    ea = ea_indexed (&cycles);
    reg_b = inst_sbc (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_f2: /* 0xf2 */
    ea = ea_extended ();
    reg_b = inst_sbc (reg_b, read8 (ea));
    cycles += 5;
    goto end;

/* addd */
op_c3: /* 0xc3 */
    set_reg_d (inst_add16 (get_reg_d (), pc_read16 ()));
    cycles += 4;
    goto end;

op_d3: /* 0xd3 */
    ea = ea_direct ();
    set_reg_d (inst_add16 (get_reg_d (), read16 (ea)));
    cycles += 6;
    goto end;

op_e3: /* 0xe3 */
    ea = ea_indexed (&cycles);
    set_reg_d (inst_add16 (get_reg_d (), read16 (ea)));
    cycles += 6;
    goto end;

op_f3: /* 0xf3 */
    ea = ea_extended ();
    set_reg_d (inst_add16 (get_reg_d (), read16 (ea)));
    cycles += 7;
    goto end;

/* andb */
op_c4: /* 0xc4 */
    reg_b = inst_and (reg_b, pc_read8 ());
    cycles += 2;
    goto end;

op_d4: /* 0xd4 */
    ea = ea_direct ();
    reg_b = inst_and (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_e4: /* 0xe4 */
    ea = ea_indexed (&cycles);
    reg_b = inst_and (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_f4: /* 0xf4 */
    ea = ea_extended ();
    reg_b = inst_and (reg_b, read8 (ea));
    cycles += 5;
    goto end;

/* bitb */
op_c5: /* 0xc5 */
    inst_and (reg_b, pc_read8 ());
    cycles += 2;
    goto end;

op_d5: /* 0xd5 */
    ea = ea_direct ();
    inst_and (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_e5: /* 0xe5 */
    ea = ea_indexed (&cycles);
    inst_and (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_f5: /* 0xf5 */
    ea = ea_extended ();
    inst_and (reg_b, read8 (ea));
    cycles += 5;
    goto end;

/* ldb */
op_c6: /* 0xc6 */
    dataBUS = reg_b = pc_read8 ();
    inst_tst8 (reg_b);
    cycles += 2;
    goto end;

op_d6: /* 0xd6 */
    ea = ea_direct ();
    reg_b = read8 (ea);
    inst_tst8 (reg_b);
    cycles += 4;
    goto end;

op_e6: /* 0xe6 */
    ea = ea_indexed (&cycles);
    reg_b = read8 (ea);
    inst_tst8 (reg_b);
    cycles += 4;
    goto end;

op_f6: /* 0xf6 */
    ea = ea_extended ();
    reg_b = read8 (ea);
    inst_tst8 (reg_b);
    cycles += 5;
    goto end;

/* eorb */
op_c8: /* 0xc8 */
    reg_b = inst_eor (reg_b, pc_read8 ());
    cycles += 2;
    goto end;

op_d8: /* 0xd8 */
    ea = ea_direct ();
    reg_b = inst_eor (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_e8: /* 0xe8 */
    ea = ea_indexed (&cycles);
    reg_b = inst_eor (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_f8: /* 0xf8 */
    ea = ea_extended ();
    reg_b = inst_eor (reg_b, read8 (ea));
    cycles += 5;
    goto end;

/* adcb */
op_c9: /* 0xc9 */
    reg_b = inst_adc (reg_b, pc_read8 ());
    cycles += 2;
    goto end;

op_d9: /* 0xd9 */
    ea = ea_direct ();
    reg_b = inst_adc (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_e9: /* 0xe9 */
    ea = ea_indexed (&cycles);
    reg_b = inst_adc (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_f9: /* 0xf9 */
    ea = ea_extended ();
    reg_b = inst_adc (reg_b, read8 (ea));
    cycles += 5;
    goto end;

/* orb */
op_ca: /* 0xca */
    reg_b = inst_or (reg_b, pc_read8 ());
    cycles += 2;
    goto end;

op_da: /* 0xda */
    ea = ea_direct ();
    reg_b = inst_or (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_ea: /* 0xea */
    ea = ea_indexed (&cycles);
    reg_b = inst_or (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_fa: /* 0xfa */
    ea = ea_extended ();
    reg_b = inst_or (reg_b, read8 (ea));
    cycles += 5;
    goto end;

/* addb */
op_cb: /* 0xcb */
    reg_b = inst_add8 (reg_b, pc_read8 ());
    cycles += 2;
    goto end;

op_db: /* 0xdb */
    ea = ea_direct ();
    reg_b = inst_add8 (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_eb: /* 0xeb */
    ea = ea_indexed (&cycles);
    reg_b = inst_add8 (reg_b, read8 (ea));
    cycles += 4;
    goto end;

op_fb: /* 0xfb */
    ea = ea_extended ();
    reg_b = inst_add8 (reg_b, read8 (ea));
    cycles += 5;
    goto end;

/* ldu */
op_ce: /* 0xce */
    reg_u = pc_read16 ();
    inst_tst16 (reg_u);
    cycles += 3;
    goto end;

op_de: /* 0xde */
    ea = ea_direct ();
    reg_u = read16 (ea);
    inst_tst16 (reg_u);
    cycles += 5;
    goto end;

op_ee: /* 0xee */
    ea = ea_indexed (&cycles);
    reg_u = read16 (ea);
    inst_tst16 (reg_u);
    cycles += 5;
    goto end;

op_fe: /* 0xfe */
    ea = ea_extended ();
    reg_u = read16 (ea);
    inst_tst16 (reg_u);
    cycles += 6;
    goto end;

/* stx */
op_9f: /* 0x9f */
    ea = ea_direct ();
    write16_cycloid (ea, reg_x);
    inst_tst16 (reg_x);
    cycles += 5;
    goto end;

op_af: /* 0xaf */
    ea = ea_indexed (&cycles);
    write16_cycloid (ea, reg_x);
    inst_tst16 (reg_x);
    cycles += 5;
    goto end;

op_bf: /* 0xbf */
    ea = ea_extended ();
    write16_cycloid (ea, reg_x);
    inst_tst16 (reg_x);
    cycles += 6;
    goto end;

/* stu */
op_df: /* 0xdf */
    ea = ea_direct ();
    write16_cycloid (ea, reg_u);
    inst_tst16 (reg_u);
    cycles += 5;
    goto end;

op_ef: /* 0xef */
    ea = ea_indexed (&cycles);
    write16_cycloid (ea, reg_u);
    inst_tst16 (reg_u);
    cycles += 5;
    goto end;

op_ff: /* 0xff */
    ea = ea_extended ();
    write16_cycloid (ea, reg_u);
    inst_tst16 (reg_u);
    cycles += 6;
    goto end;

/* std */
op_dd: /* 0xdd */
    ea = ea_direct ();
    write16_cycloid (ea, get_reg_d ());
    inst_tst16 (get_reg_d ());
    cycles += 5;
    goto end;

op_ed: /* 0xed */
    ea = ea_indexed (&cycles);
    write16_cycloid (ea, get_reg_d ());
    inst_tst16 (get_reg_d ());
    cycles += 5;
    goto end;

op_fd: /* 0xfd */
    ea = ea_extended ();
    write16_cycloid (ea, get_reg_d ());
    inst_tst16 (get_reg_d ());
    cycles += 6;
    goto end;

/* subb */
op_c0: /* 0xC0 */
    reg_b = inst_sub8 (reg_b, pc_read8 ()) & 0xff;
    cycles += 2;
    goto end;

op_d0: /* 0xD0 */
    ea = ea_direct ();
    reg_b = inst_sub8 (reg_b, read8 (ea)) & 0xff;
    cycles += 4;
    goto end;

op_e0: /* 0xE0 */
    ea = ea_indexed (&cycles);
    reg_b = inst_sub8 (reg_b, read8 (ea)) & 0xff;
    cycles += 4;
    goto end;

op_f0: /* 0xF0 */
    ea = ea_extended ();
    reg_b = inst_sub8 (reg_b, read8 (ea)) & 0xff;
    cycles += 5;
    goto end;


/* default / illegal opcode */
op_default:
    /* keine Aktion, nur Zyklen behalten */
    goto end;

end:
    return cycles;
}

