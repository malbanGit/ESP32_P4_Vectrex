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
DRAM_ATTR int addressBUS = 0;
DRAM_ATTR unsigned char dataBUS = 0;

// static int PRE_CLR_STEPS = 4;
// static int POST_CLR_ADDSTEPS = -1;

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

static IRAM_ATTR einline unsigned get_cc (unsigned flag)
{
	return (reg_cc / flag) & 1;
}

/* set a particular condition code to either 0 or 1.
 * value parameter must be either 0 or 1.
 */

static IRAM_ATTR einline void set_cc (unsigned flag, unsigned value)
{
	reg_cc = value?(reg_cc | flag):(reg_cc & ~flag);
}

/* test carry */

static IRAM_ATTR einline unsigned test_c (unsigned i0, unsigned i1,
								unsigned r, unsigned sub)
{
	return (((((i0 | i1) & ~r) | (i0 & i1)) & 0x80) == 0x80) ^ (sub);
}

/* test negative */

static IRAM_ATTR einline unsigned test_n (unsigned r)
{
	return (r & 0x80) == 0x80;
}

/* test for zero in lower 8 bits */

static IRAM_ATTR einline unsigned test_z8 (unsigned r)
{
	return (r&0xff) == 0;
}

/* test for zero in lower 16 bits */

static IRAM_ATTR einline unsigned test_z16 (unsigned r)
{
	return (r&0xffff) == 0;
}

/* overflow is set whenever the sign bits of the inputs are the same
 * but the sign bit of the result is not same as the sign bits of the
 * inputs.
 */

static IRAM_ATTR einline unsigned test_v (unsigned i0, unsigned i1, unsigned r)
{
	return (((~(i0 ^ i1)) & (i0 ^ r)) & 0x80) == 0x80;
}

static IRAM_ATTR einline unsigned get_reg_d (void)
{
	return (((reg_a << 8)&0xff00) | (reg_b & 0xff));
}

static IRAM_ATTR einline void set_reg_d (unsigned value)
{
	reg_a = (value >> 8) & 0xff;
	reg_b = value & 0xff;
}

/* read a byte ... the returned value has the lower 8-bits set to the byte
 * while the upper bits are all zero.
 */

static IRAM_ATTR einline unsigned read8 (unsigned address)
{
	return (*e6809_read8) (address & 0xffff)&0xff;
}

/* write a byte ... only the lower 8-bits of the unsigned data
 * is written. the upper bits are ignored.
 */

static IRAM_ATTR einline void write8 (unsigned address, unsigned data)
{
	(*e6809_write8) (address & 0xffff, (unsigned char) data);
}

static IRAM_ATTR einline unsigned read16 (unsigned address)
{
	unsigned datahi = read8 (address);
	unsigned datalo = read8 (address + 1);
	return (datahi << 8) | datalo;
}
static IRAM_ATTR einline unsigned read16_cycloid (unsigned address)
{
	unsigned datahi = read8 (address);
//    //vecx_intermediateStepsUC(1);
	unsigned datalo = read8 (address + 1);
	return (datahi << 8) | datalo;
}

static IRAM_ATTR einline void write16 (unsigned address, unsigned data)
{
	write8 (address, data >> 8);
	write8 (address + 1, data);
}
static IRAM_ATTR einline void write16_cycloid (unsigned address, unsigned data)
{
	write8 (address, data >> 8);
//    //vecx_intermediateStepsUC(1);
	write8 (address + 1, data);
}

static IRAM_ATTR einline void push8 (unsigned *sp, unsigned data)
{
	(*sp)--;
	write8 (*sp, data);
}

static IRAM_ATTR einline unsigned pull8 (unsigned *sp)
{
	unsigned	data = read8 (*sp);
	(*sp)++;

	return data;
}

static IRAM_ATTR einline void push16 (unsigned *sp, unsigned data)
{
	push8 (sp, data);
	push8 (sp, data >> 8);
}

static IRAM_ATTR einline unsigned pull16 (unsigned *sp)
{
	unsigned datahi = pull8 (sp);
	unsigned datalo = pull8 (sp);

	return (datahi << 8) | datalo;
}

/* read a byte from the address pointed to by the pc */

static IRAM_ATTR einline unsigned pc_read8 (void)
{
	unsigned data = read8 (reg_pc);
    reg_pc=(reg_pc+1)&0xffff;

	return data;
}

/* read a word from the address pointed to by the pc */

static IRAM_ATTR einline unsigned pc_read16 (void)
{
	unsigned data = read16 (reg_pc);
    reg_pc=(reg_pc+2)&0xffff;

	return data;
}

/* sign extend an 8-bit quantity into a 16-bit quantity */

static IRAM_ATTR einline unsigned sign_extend (unsigned data)
{
	return (~(data & 0x80) + 1) | (data & 0xff);
}

/* direct addressing, upper byte of the address comes from
 * the direct page register, and the lower byte comes from the
 * instruction itself.
 */

static IRAM_ATTR einline unsigned ea_direct (void)
{
	return (reg_dp << 8) | pc_read8 ();
}

/* extended addressing, address is obtained from 2 bytes following
 * the instruction.
 */

static IRAM_ATTR einline unsigned ea_extended (void)
{
	return addressBUS = pc_read16 ();
}

/* indexed addressing */

static IRAM_ATTR einline unsigned ea_indexed (unsigned *cycles)
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

static IRAM_ATTR einline unsigned inst_neg (unsigned data)
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

static IRAM_ATTR einline unsigned inst_com (unsigned data)
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

static IRAM_ATTR einline unsigned inst_lsr (unsigned data)
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

static IRAM_ATTR einline unsigned inst_ror (unsigned data)
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

static IRAM_ATTR einline unsigned inst_asr (unsigned data)
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

static IRAM_ATTR einline unsigned inst_asl (unsigned data)
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

static IRAM_ATTR einline unsigned inst_rol (unsigned data)
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

static IRAM_ATTR einline unsigned inst_dec (unsigned data)
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

static IRAM_ATTR einline unsigned inst_inc (unsigned data)
{
	unsigned r = (data + 1)&0xff;

    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = (r == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = (data==0x7f)?(reg_cc | FLAG_V):(reg_cc & ~FLAG_V);


	return r;
}

/* instruction: tst */

static IRAM_ATTR einline void inst_tst8 (unsigned data)
{
    reg_cc = ((data & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((data&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = (reg_cc & ~FLAG_V);
}

static IRAM_ATTR einline void inst_tst16 (unsigned data)
{
    reg_cc =  (((data) & 0x8000) == 0x8000) ?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((data&0xffff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = (reg_cc & ~FLAG_V);
}

/* instruction: clr */

static IRAM_ATTR einline void inst_clr (void)
{
    reg_cc = (reg_cc & ~FLAG_N);
    reg_cc = (reg_cc | FLAG_Z);
    reg_cc = (reg_cc & ~FLAG_V);
    reg_cc = (reg_cc & ~FLAG_C);
}

/* instruction: suba/subb */

static IRAM_ATTR einline unsigned inst_sub8 (unsigned data0, unsigned data1)
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

static IRAM_ATTR einline unsigned inst_sbc (unsigned data0, unsigned data1)
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

static IRAM_ATTR einline unsigned inst_and (unsigned data0, unsigned data1)
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

static IRAM_ATTR einline unsigned inst_eor (unsigned data0, unsigned data1)
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

static IRAM_ATTR einline unsigned inst_adc (unsigned data0, unsigned data1)
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

static IRAM_ATTR einline unsigned inst_or (unsigned data0, unsigned data1)
{
	unsigned     r = data0 | data1;

    reg_cc = ((r & 0x80) == 0x80)?(reg_cc | FLAG_N):(reg_cc & ~FLAG_N);
    reg_cc = ((r&0xff) == 0)?(reg_cc | FLAG_Z):(reg_cc & ~FLAG_Z);
    reg_cc = (reg_cc & ~FLAG_V);

	return r;
}

/* instruction: adda/addb */

static IRAM_ATTR einline unsigned inst_add8 (unsigned data0, unsigned data1)
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

static IRAM_ATTR einline unsigned inst_add16 (unsigned data0, unsigned data1)
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

static IRAM_ATTR einline unsigned inst_sub16 (unsigned data0, unsigned data1)
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

static IRAM_ATTR einline void inst_bra8 (unsigned test, unsigned op, unsigned *cycles)
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

static IRAM_ATTR einline void inst_bra16 (unsigned test, unsigned op, unsigned *cycles)
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

static IRAM_ATTR einline void inst_psh (unsigned op, unsigned *sp,
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

static IRAM_ATTR einline void inst_pul (unsigned op, unsigned *sp, unsigned *osp,
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

static IRAM_ATTR einline unsigned exgtfr_read (unsigned reg)
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

static IRAM_ATTR einline void exgtfr_write (unsigned reg, unsigned data)
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

static IRAM_ATTR einline void inst_exg (void)
{
	unsigned op, tmp;

	op = pc_read8 ();

	tmp = exgtfr_read (op & 0xf);
	exgtfr_write (op & 0xf, exgtfr_read ((op >> 4) &0xf));
	exgtfr_write (op >> 4, tmp);
}


/* instruction: tfr */

static IRAM_ATTR einline void inst_tfr (void)
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverride-init"

// the second might be needed for imager or lightpen
//IRAM_ATTR unsigned e6809_sstep (unsigned irq_i, unsigned irq_f)
//IRAM_ATTR unsigned e6809_sstep (unsigned irq_i)
/*
IRAM_ATTR static  unsigned e6809_sstep ()
{
	unsigned icycles;
#include "6809Step.i"
    return icycles;
}
*/