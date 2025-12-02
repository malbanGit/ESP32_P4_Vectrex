#include <stdio.h>
#include "e6809.h"
#include "e8910.h"
#include "vecx.h"
#include "cart.h"
#include "system.h"

#include "esp_attr.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define einline __inline

DRAM_ATTR static long cyclesRunning = 0;
DRAM_ATTR static int thisWaitRecal = 0;
DRAM_ATTR static long lastWaitRecal=0;
DRAM_ATTR static long lastSyncCycles = 0;
DRAM_ATTR static int syncImpulse = 0;

DRAM_ATTR static int config_autoSync = 1;
DRAM_ATTR static int sig_ramp = 0; 
DRAM_ATTR static int sig_blank = 0; // moved out of emu_loop for lightpen access
DRAM_ATTR static int sig_zero = 0;
DRAM_ATTR static int alternate = 0;
DRAM_ATTR static int currentBank = 0;
DRAM_ATTR static int currentIRQ = 1;
DRAM_ATTR static int currentPB6 = 1;
DRAM_ATTR static int pb6_in = 0x40; // 0 or 0x40 in from external
DRAM_ATTR static int pb6_out = 0x40; // out from vectrex
DRAM_ATTR static int BANK_MAX = 1;
//void timerAddItem(int value, void *destination, int t);



#define EMU_TIMER 20 /* the emulators heart beats at 20 milliseconds */
#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 1280

DRAM_ATTR static long scl_factorx;
DRAM_ATTR static long scl_factory;
DRAM_ATTR static long offx;
DRAM_ATTR static long offy;

//static char *romfilename = "rom.dat";
DRAM_ATTR static char *cartfilename = NULL;

//unsigned char rom[8192];
DRAM_ATTR unsigned char *rom = (unsigned char *)bios_data;
DRAM_ATTR static unsigned char ram[1024];

/* the sound chip registers */

DRAM_ATTR unsigned snd_regs[16];
DRAM_ATTR static unsigned snd_select;

/* the via 6522 registers */

DRAM_ATTR static unsigned via_ora;
DRAM_ATTR static unsigned via_orb;
DRAM_ATTR static unsigned via_ddra;
DRAM_ATTR static unsigned via_ddrb;
DRAM_ATTR static unsigned via_t1on;  /* is timer 1 on? */
DRAM_ATTR static unsigned via_t1int; /* are timer 1 interrupts allowed? */
DRAM_ATTR static unsigned via_t1c;
DRAM_ATTR static unsigned via_t1ll;
DRAM_ATTR static unsigned via_t1lh;
DRAM_ATTR static unsigned via_t1pb7; /* timer 1 controlled version of pb7 */
DRAM_ATTR static unsigned via_t2on;  /* is timer 2 on? */
DRAM_ATTR static unsigned via_t2int; /* are timer 2 interrupts allowed? */
DRAM_ATTR static unsigned via_t2c;
DRAM_ATTR static unsigned via_t2ll;
DRAM_ATTR static unsigned via_sr;
DRAM_ATTR static unsigned via_srb;   /* number of bits shifted so far */
DRAM_ATTR static unsigned via_src;   /* shift counter */
DRAM_ATTR static unsigned via_srclk;
DRAM_ATTR static unsigned via_acr;
DRAM_ATTR static unsigned via_pcr;
DRAM_ATTR static unsigned via_ifr;
DRAM_ATTR static unsigned via_ier;
DRAM_ATTR static unsigned old_via_ca1;
DRAM_ATTR static unsigned via_ca1;
DRAM_ATTR static unsigned via_ca2;
DRAM_ATTR static unsigned via_cb2h;  /* basic handshake version of cb2 */
DRAM_ATTR static unsigned via_cb2s;  /* version of cb2 controlled by the shift register */

/* analog devices */

DRAM_ATTR static unsigned alg_sel;
DRAM_ATTR static unsigned alg_ramping;
DRAM_ATTR static signed alg_DAC;  /* z sample and hold */
DRAM_ATTR static signed alg_ssh;
DRAM_ATTR static unsigned alg_rsh;  /* zero ref sample and hold */
DRAM_ATTR static unsigned alg_xsh;  /* x sample and hold */
DRAM_ATTR static unsigned alg_ysh;  /* y sample and hold */
DRAM_ATTR static unsigned alg_zsh;  /* z sample and hold */
DRAM_ATTR unsigned alg_jch0;		  /* joystick direction channel 0 */
DRAM_ATTR unsigned alg_jch1;		  /* joystick direction channel 1 */
DRAM_ATTR unsigned alg_jch2;		  /* joystick direction channel 2 */
DRAM_ATTR unsigned alg_jch3;		  /* joystick direction channel 3 */
DRAM_ATTR static unsigned alg_jsh;  /* joystick sample and hold */

DRAM_ATTR static unsigned alg_compare;

DRAM_ATTR static long alg_dx;     /* delta x */
DRAM_ATTR static long alg_dy;     /* delta y */
DRAM_ATTR static long alg_curr_x; /* current x position */
DRAM_ATTR static long alg_curr_y; /* current y position */

enum {
	VECTREX_PDECAY	= 30,      /* phosphor decay rate */

	/* number of 6809 cycles before a frame redraw */

	FCYCLES_INIT    = VECTREX_MHZ / VECTREX_PDECAY,

	/* max number of possible vectors that maybe on the screen at one time.
	 * one only needs VECTREX_MHZ / VECTREX_PDECAY but we need to also store
	 * deleted vectors in a single table
	 */

	VECTOR_CNT		= VECTREX_MHZ / VECTREX_PDECAY,
};

DRAM_ATTR static unsigned alg_vectoring; /* are we drawing a vector right now? */
DRAM_ATTR static long alg_vector_x0;
DRAM_ATTR static long alg_vector_y0;
DRAM_ATTR static long alg_vector_x1;
DRAM_ATTR static long alg_vector_y1;
DRAM_ATTR static long alg_vector_dx;
DRAM_ATTR static long alg_vector_dy;
DRAM_ATTR static unsigned char alg_vector_color;


// we must put following data into PSRAM, the internal RAM is not enough!

// this could be optimized with a linked list!
// I never encountered more than 10
// if more timers are needed, core will EXIT!!!
// this is not dynamic
#define MAX_TIMER 10 // only ten needed in normal setup, this is trial TODO
typedef struct TimerItem_s 
{
	signed int countDown;
	unsigned char valueToSet;
	int *whereToSet;
	int type;
	int active;
} TimerItem;
DRAM_ATTR  TimerItem timerItemArray[MAX_TIMER]; // should be saved

// these are types, and Array Index, both!
enum
{
    TIMER_ACTION_NONE = 0,
    TIMER_ZERO = 1,
    TIMER_BLANK_ON_CHANGE = 2,
    TIMER_BLANK_OFF_CHANGE = 3,
    TIMER_RAMP_CHANGE = 4,
    TIMER_MUX_Y_CHANGE = 5,
    TIMER_MUX_S_CHANGE = 6,
    TIMER_MUX_Z_CHANGE = 7,
    TIMER_MUX_R_CHANGE = 8,
    TIMER_XSH_CHANGE = 9,
    TIMER_LIGHTPEN = 10,
    TIMER_RAMP_OFF_CHANGE = 11,
    TIMER_MUX_SEL_CHANGE = 12,
    TIMER_SHIFT = 13,
    TIMER_T1 = 14,
    TIMER_T2 = 15,
	TIMER_SHIFT_WRITE = 13+1024, // the delay is the normal "SHIFT"
    TIMER_SHIFT_READ = 13+2048, // the delay is the normal "SHIFT"
};
DRAM_ATTR static int DELAYS[]={
	0,  // TIMER_ACTION_NONE = 0,
	5,  // TIMER_ZERO = 1,
	0,  // TIMER_BLANK_ON_CHANGE = 2,
	0,  // TIMER_BLANK_OFF_CHANGE = 3,
	12, // TIMER_RAMP_CHANGE = 4,
	14, // TIMER_MUX_Y_CHANGE = 5,
	0,  // TIMER_MUX_S_CHANGE = 6,
	0,  // TIMER_MUX_Z_CHANGE = 7,
	0,  // TIMER_MUX_R_CHANGE = 8,
	15, // TIMER_XSH_CHANGE = 9,
	0,  // TIMER_LIGHTPEN = 10,
	15, // TIMER_RAMP_OFF_CHANGE = 11,
	1,  // TIMER_MUX_SEL_CHANGE = 12, 
	0,  // TIMER_SHIFT = 13, 
	0,  // TIMER_T1 = 14, 
	0   // TIMER_T2 = 15,
	}; // no need to be saved


DRAM_ATTR static long fcycles;


void emu_begin_frame(void);
void emu_end_frame(void);
void emu_draw_line(int x0, int y0, int x1, int y1, uint8_t brightness); // For ESP32
void vecx_emu (long cycles);

IRAM_ATTR static einline void readevents()
{
}

void osint_emuloop()
{
	vecx_emu((VECTREX_MHZ / 1000) * EMU_TIMER);
	readevents();
}

void resize(int width, int height){
	long sclx, scly;

	long screenx = width;
	long screeny = height;

	sclx = ALG_MAX_X / SCREEN_WIDTH;
	scly = ALG_MAX_Y / SCREEN_HEIGHT;

	scl_factorx = -sclx;
	scl_factory = -scly;

	offx = (screenx - ALG_MAX_X / scl_factorx) / 2;
	offy = (screeny - ALG_MAX_Y / scl_factory) / 2;
}

static void init()
{
/*	
	FILE *f;
	if(!(f = fopen(romfilename, "rb"))){
		perror(romfilename);
		exit(EXIT_FAILURE);
	}
	if(fread(rom, 1, sizeof (rom), f) != sizeof (rom)){
		printf("Invalid rom length\n");
		exit(EXIT_FAILURE);
	}
	fclose(f);
*/
	cartInit(cartfilename);
}

int vecx_init()
{
	init();

	resize(SCREEN_WIDTH, SCREEN_HEIGHT);

	vecx_reset();
	e8910_init_sound();
//	osint_emuloop();
//	e8910_done_sound();
	return 0;
}

IRAM_ATTR static einline  void alg_addline (long x0, long y0, long x1, long y1, unsigned char color)
{
	emu_draw_line(offx + x0 / scl_factorx, offy +y0 / scl_factory, offx + x1 / scl_factorx, offy + y1 / scl_factory, color*2); // For ESP32
	return;
}

static IRAM_ATTR einline void timerAddItem(int value, void *destination, int t)
{
	for (int i=0;i<MAX_TIMER;i++)
	{
		if (timerItemArray[i].active == 0)
		{
			timerItemArray[i].active = 1;
			timerItemArray[i].valueToSet = value&0xff;
			timerItemArray[i].whereToSet = destination;
			timerItemArray[i].type = t; 
			timerItemArray[i].countDown = DELAYS[(t&0xff)];
			return;
		}
	}
	printf("NOT ENOUGH TIMERS! PANIC!!!\n");
	exit(3);
}

IRAM_ATTR static einline void doCheckRamp(int fromOrbWrite)
{
	if (!fromOrbWrite)
	{
		if ((via_acr & 0x80)!=0) 
		{
			if (via_t1pb7==0)
				timerAddItem(via_t1pb7, &sig_ramp, TIMER_RAMP_CHANGE);
			else
				timerAddItem(via_t1pb7, &sig_ramp, TIMER_RAMP_OFF_CHANGE);
		} 
	}
	else
	{
		if ((via_acr & 0x80)==0) 
		{
			if ((via_orb & 0x80) == 0)
				timerAddItem(via_orb & 0x80 , &sig_ramp, TIMER_RAMP_CHANGE);
			else
				timerAddItem(via_orb & 0x80, &sig_ramp, TIMER_RAMP_OFF_CHANGE);
		}
	}
}


// this might be done "nicer" - 
// didn'T think about it much.... it works... so be it...
IRAM_ATTR static einline  signed int makeSigned(unsigned char data)
{
   if (data > 127) return -(256-data);
   return data;
}

IRAM_ATTR static einline  unsigned char makeUnsigned(signed int data)
{
   return data&0xff;
}

/* update IRQ and bit-7 of the ifr register after making an adjustment to
 * ifr.
 */

// assuming
// 64k carts are two banks of 32k
// 256k carts (VB) are 4 banks of 64k
void setBank()
{
	currentBank = 0;
	if (BANK_MAX == 1) return;
	if (BANK_MAX >= 2)
	{
		if (currentPB6) currentBank++;
	}
	if (BANK_MAX >= 4)
	{
		if (currentIRQ) currentBank+=2;
	}
}
void setPB6FromVectrex(int tobe_via_orb, int  tobe_via_ddrb, int orbInitiated)
{
	if (BANK_MAX <= 1) return;

	int npb6 = tobe_via_orb & tobe_via_ddrb & 0x40; // all output (0x40)
	if ((tobe_via_ddrb & 0x40) == 0x00)  npb6 = npb6 | 0x40; // all input (0x40)
	pb6_out = npb6;
	currentPB6 = (npb6 != 0);
	setBank();
}
void setPB6FromExternal(int b)
{
	if (b)
		pb6_in = 0x40;
	else
		pb6_in = 0x00;

}
	
void setIRQFromVectrex(int irq)
{
	if (BANK_MAX <= 2) return;
	currentIRQ = !irq;
	setBank();
}

IRAM_ATTR static einline  void int_update ()
{
	if ( (((via_ifr & 0x7f) & (via_ier & 0x7f))) != 0   ) 
	{
		via_ifr |= 0x80;
	} 
	else 
	{
		via_ifr &= 0x7f;
	}
	setIRQFromVectrex(((via_ifr&0x80) !=0 ));
}

/* update the various analog values when orb is written. */
IRAM_ATTR static einline void doCheckMultiplexer()
{
   /* compare the current joystick direction with a reference */
	switch (via_orb & 0x06) 
	{
		case 0x00:
			alg_jsh = makeSigned(alg_jch0); 
			break;
		case 0x02:
			alg_jsh = makeSigned(alg_jch1); 
			break;
		case 0x04:
			alg_jsh = makeSigned(alg_jch2); 
			break;
		case 0x06:
			alg_jsh = makeSigned(alg_jch3); 
			break;
	}                
	// 0-255, 128 middle
   if ((makeUnsigned((signed int)alg_jsh)) > ((via_ora&0xff)^0x80))
      alg_compare = 0x20;
   else
      alg_compare = 0;

	if ((via_orb & 0x01) != 0) return;
	
	/* MUX has been enabled, state changed! */
	switch (alg_sel & 0x06) 
	{
		case 0x00:
			/* demultiplexor is on */
			timerAddItem(alg_DAC, &alg_ysh, TIMER_MUX_Y_CHANGE);
			break;
		case 0x02:
			/* demultiplexor is on */
			timerAddItem(alg_DAC, 0, TIMER_MUX_R_CHANGE);
			break;
		case 0x04:
			/* demultiplexor is on */
			timerAddItem(alg_DAC , &alg_zsh, TIMER_MUX_Z_CHANGE);
// todo
			//			intensityDrift = 0;
			break;
		case 0x06:
			/* sound output line */
			timerAddItem(alg_DAC , &alg_ssh, TIMER_MUX_S_CHANGE);
			break;
			
	}
}

void initTimerArray()
{
	for (int i=0;i<MAX_TIMER;i++)
	{
		timerItemArray[i].countDown = 0;
		timerItemArray[i].valueToSet = 0;
		timerItemArray[i].whereToSet = 0;
		timerItemArray[i].type = TIMER_ACTION_NONE;
		timerItemArray[i].active = 0;
	}
}
//return 1 on success
//return 0 on fail
IRAM_ATTR einline int removeTimer(TimerItem *t)
{
	for (int i=0;i<MAX_TIMER;i++)
	{
		if (&timerItemArray[i] == t)
		{
			timerItemArray[i].active = 0;
			return 1;
		}
	}
	return 0;
}

static IRAM_ATTR einline void timerDoStep()
{
	for (int i=0;i<MAX_TIMER;i++)
	{
		if (timerItemArray[i].active == 0) continue;
		timerItemArray[i].countDown--;
		if (timerItemArray[i].countDown <=0)
		{
			if (timerItemArray[i].type == TIMER_SHIFT_READ)
			{
				alternate = 1;
				//lastShiftTriggered = cyclesRunning;
				via_ifr &= 0xfb; /* remove shift register interrupt flag */
				via_srb = 0;
				via_srclk = 1;
				int_update ();
			} 
			else if (timerItemArray[i].type == TIMER_SHIFT_WRITE)
			{
				alternate = 1;
				//via_stalling = 0;
				//lastShiftTriggered = cyclesRunning;
				via_sr = timerItemArray[i].valueToSet;
				via_ifr &= 0xfb; /* remove shift register interrupt flag */
				via_srb = 0;
				via_srclk = 1;
				int_update ();
			}
			else if (timerItemArray[i].type == TIMER_T1)
			{
				via_t1on = 1; /* timer 1 starts running */
				via_t1lh = timerItemArray[i].valueToSet;
				via_t1c = (via_t1lh << 8) | via_t1ll;
				via_ifr &= 0xbf; /* remove timer 1 interrupt flag */
				via_t1int = 1;

				via_t1pb7 = 0;
				doCheckRamp(0);
				int_update ();
			}
			else if (timerItemArray[i].type == TIMER_T2)
			{
				via_t2c = ((timerItemArray[i].valueToSet) << 8) | via_t2ll;
				via_t2c += 0; // hack, it seems vectrex (via) takes two cycles to "process" the setting...
				via_ifr &= 0xdf;
				via_t2on = 1; /* timer 2 starts running */
				via_t2int = 1;
				int_update ();
			}
			
			else if (timerItemArray[i].type == TIMER_MUX_R_CHANGE)
			{
				//noiseCycles = cyclesRunning;
				// setDigitalVoltage(timerItemArray[i].valueToSet); 
				alg_rsh = alg_xsh;
			}
			else if (timerItemArray[i].whereToSet != 0)   
			{
				if (timerItemArray[i].type == TIMER_RAMP_OFF_CHANGE) 
				{
					// difference of 3 is to much, but we have no "smaller" unit than
					// cycles ticks
					// therefor we calculate a fraction on ticks to be as exact as possible
					// the fraction here is not KNOWN, it is
					// experimented
					// analog curcuits don't really care about cycles...
//					if ((t.whereToSet& 0xff) != (t.valueToSet & 0xff))
//					{
//						rampOffFraction = 1;
//					}
				}
				else if (timerItemArray[i].type == TIMER_RAMP_CHANGE) 
				{
					// difference of 3 is to much, but we have no "smaller" unit than
					// cycles ticks
					// therefor we calculate a fraction on ticks to be as exact as possible
					// the fraction here is not KNOWN, it is
					// experimented
					// analog curcuits don't really care about cycles...
//					if ((t.whereToSet& 0xff) != (t.valueToSet & 0xff))
//					{
//						rampOnFraction = 1;
//					}
				}
/*                    
				// ATTENTION!
				// it looks like MUX SEL has a time - offset
				// but the value that is used to "transport" to the receiving SH
				// is the one, when "timing" expires, not when the timer is
				// set
				// luckily that is ALLWAYS via_ora
				// so we can take it here directly and ignore the value that
				// is passed to timing!
				if (t.type == TIMER_MUX_Y_CHANGE)
				{
					// test above "theory" with Y
					t.whereToSet = via_ora & 0xff;
				}
*/
				if (timerItemArray[i].type != TIMER_MUX_Z_CHANGE) // Z must not be negative
				{
					*timerItemArray[i].whereToSet = makeSigned(timerItemArray[i].valueToSet);
				}
				else
				{
					*timerItemArray[i].whereToSet = timerItemArray[i].valueToSet;
				}

				if (timerItemArray[i].type == TIMER_MUX_SEL_CHANGE)
					doCheckMultiplexer();
			
			}
			timerItemArray[i].active = 0;
		}
	}
}


/* update the snd chips internal registers when via_ora/via_orb changes */
IRAM_ATTR static einline void snd_update(int command)
{
   switch (via_orb & 0x18)
   {
      case 0x00:
         /* the sound chip is disabled */
         break;
      case 0x08:
         /* the sound chip is sending data */
		if (command)
			via_ora = e8910_read(snd_select); // this is for joystick - dummy for now!
         break;
      case 0x10:
         /* the sound chip is recieving data */
		 if (command)
		 {
			 if (snd_select != 14)
			 {
				snd_regs[snd_select] = via_ora;
				e8910_write(snd_select, via_ora);
			 }
		 }

         break;
      case 0x18:
         /* the sound chip is latching an address */

         if ((via_ora & 0xf0) == 0x00)
            snd_select = via_ora & 0x0f;

         break;
   }
	if ((via_orb & 0x07) == 0x06) // SEL == 11 -> Sound, Mux ==0 meaning ON
	{
		// dac is sending data to audio hardware
		// since we are used to do audio in PSG anyway, we send the sampled data there to a "dummy" register
		// data is via_ora
		// dummy register, write directly to audio line buffer in psg emulation!
		e8910_write(255, alg_ssh);
	}
}

IRAM_ATTR unsigned char read8 (unsigned address)
{
   unsigned char data = 0;

   /* rom */
   if ((address & 0xe000) == 0xe000)
      data = rom[address & 0x1fff];
   
   else if ((address & 0xe000) == 0xc000)
   {
      /* ram */
      if (address & 0x800)
         data = ram[address & 0x3ff];
      else if (address & 0x1000)
      {
         /* io */

         switch (address & 0xf)
         {
            case 0x0:
				/* compare signal is an input so the value does not come from
				 * via_orb.
				 */
				if ((via_acr & 0x80) !=0)
				{
					/* timer 1 has control of bit 7 */
					data = ((via_orb & 0x5f) | alg_compare | via_t1pb7);
				} 
				else 
				{
					/* bit 7 is being driven by via_orb */
					data = ((via_orb & 0xdf) | alg_compare);
				}
				if ((via_ddrb & 0x40) == 0) // pb6 is input
				{
					data = data & (0xff-0x40); // ensure pb6 =0
					data = data | (pb6_in); // ensure pb6 in value
				}
				else
				{
				}
				return data&0xff;
               break;
            case 0x1:
               /* register 1 also performs handshakes if necessary */

               /* if ca2 is in pulse mode or handshake mode, then it
                * goes low whenever ira is read.
                */
               if ((via_pcr & 0x0e) == 0x08)
			   {
                    via_ca2 = 0;
				    timerAddItem(via_ca2,&sig_zero, TIMER_ZERO);
			   }

               via_ifr = via_ifr & (0xff-0x02); //
     		   setIRQFromVectrex(((via_ifr&0x80) !=0 )); // 
               /* fall through */

            case 0xf:
               /* the snd chip is driving port a */
               if ((via_orb & 0x18) == 0x08)
                  data = (unsigned char) snd_regs[snd_select];
               else
                  data = (unsigned char) via_ora;

               break;
            case 0x2:
               data = (unsigned char) via_ddrb;
               break;
            case 0x3:
               data = (unsigned char) via_ddra;
               break;
            case 0x4:
				/* T1 low order counter */
				data = via_t1c;
				via_ifr &= 0xbf; /* remove timer 1 interrupt flag */
//                        via_t1int = 0; // THIS WAS original - and is wrong!
				via_t1int = 1;
				int_update ();
				return data&0xff;
               break;
            case 0x5:
               /* T1 high order counter */
               data = (unsigned char) (via_t1c >> 8);
               break;
            case 0x6:
               /* T1 low order latch */
               data = (unsigned char) via_t1ll;
               break;
            case 0x7:
               /* T1 high order latch */
               data = (unsigned char) via_t1lh;
               break;
            case 0x8:
               /* T2 low order counter */
               data      = (unsigned char) via_t2c;
				via_ifr &= 0xdf; /* remove timer 2 interrupt flag */
				via_t2int = 1;
				int_update ();
               break;
            case 0x9:
               /* T2 high order counter */
               data = (unsigned char) (via_t2c >> 8);
               break;
            case 0xa:
               data = (unsigned char) via_sr&0xff;
               timerAddItem(via_sr, 0, TIMER_SHIFT_READ);
               break;
            case 0xb:
               data = (unsigned char) via_acr;
               break;
            case 0xc:
               data = (unsigned char) via_pcr;
               break;
            case 0xd:
               /* interrupt flag register */

               data = (unsigned char) via_ifr;
               break;
            case 0xe:
               /* interrupt enable register */

               data = (unsigned char) (via_ier | 0x80);
               break;
         }
      }
   }
	// 
	else if( address < 0xc000 )
	{
		/* Todo
	   if (BANK_MAX<4) 
		   data = cart[address+(currentBank *32768)] & 0xff; // 
	   else 
		   data = cart[address+(currentBank *65536)] & 0xff; // 
		*/
	}
   else
      data = 0xff;
   return data;
}


IRAM_ATTR void write8 (unsigned address, unsigned char data)
{
   /* rom */
	if ((address & 0xe000) == 0xe000) { }
	else if ((address & 0xe000) == 0xc000)
   {
      /* it is possible for both ram and io to be written at the same! */

      if (address & 0x800)
         ram[address & 0x3ff] = data;

      if (address & 0x1000)
      {
         switch (address & 0xf)
         {
            case 0x0:
				setPB6FromVectrex(data, via_ddrb, 1); // 
				if ((data & 0x7) != (via_orb & 0x07)) // check if state of mux sel changed
				{
					timerAddItem(data, &alg_sel, TIMER_MUX_SEL_CHANGE);
				}
				via_orb = data;

               snd_update (1);

				if ((via_pcr & 0xe0) == 0x80) 
				{
					/* if cb2 is in pulse mode or handshake mode, then it
					 * goes low whenever orb is written.
					 */
					via_cb2h = 0;
					timerAddItem(via_cb2h, &sig_blank, TIMER_BLANK_ON_CHANGE);
	
				}
				doCheckRamp(1);

               break;
            case 0x1:
               /* register 1 also performs handshakes if necessary */

               /* if ca2 is in pulse mode or handshake mode, then it
                * goes low whenever ora is written.
                */
               if ((via_pcr & 0x0e) == 0x08)
			   {
					via_ca2 = 0;
					timerAddItem(via_ca2,&sig_zero, TIMER_ZERO);
			   }
				via_ifr = via_ifr & (0xff-0x02); // clear ca1 interrupt
				setIRQFromVectrex(((via_ifr&0x80) !=0 ));

               /* fall through */

            case 0xf:
               via_ora = data;
			   alg_DAC = makeSigned(data);


               /* output of port a feeds directly into the dac which then
                * feeds the x axis sample and hold.
                */
				timerAddItem(alg_DAC, &alg_xsh, TIMER_XSH_CHANGE);
				doCheckMultiplexer();
               snd_update (0);

               break;
            case 0x2:
				setPB6FromVectrex(via_orb, data, 0); // 
               via_ddrb = data;
               break;
            case 0x3:
               via_ddra = data;
               break;
            case 0x4:
               /* T1 low order counter */
               via_t1ll = data;

               break;
            case 0x5:
               /* T1 high order counter */
               timerAddItem(data,0, TIMER_T1);
               break;
            case 0x6:
               /* T1 low order latch */

               via_t1ll = data;
               break;
            case 0x7:
               /* T1 high order latch */

               via_t1lh = data;
               break;
            case 0x8:
               /* T2 low order latch */

               via_t2ll = data;
               break;
            case 0x9:
               /* T2 high order latch/counter */
               timerAddItem(data,0, TIMER_T2);
               break;
            case 0xa:
				timerAddItem(data, 0, TIMER_SHIFT_WRITE);
               break;
            case 0xb:
				if ((via_acr & 0x1c) != (data & 0x1c))
				{
					if ((data & 0x1c) == 0) // shift reg is switched off - so take the manual value
					{
					}
					else // use the last shift
					{
						timerAddItem(0, &sig_blank, TIMER_BLANK_ON_CHANGE);
					}
				}
				if ((via_acr & 0xc0) != (data & 0xc0))
				{
					via_acr = data;
					doCheckRamp(!((via_acr&0x80) == 0x80));
				}
				
				via_acr = data;
               break;
            case 0xc:
				via_pcr = data;
				if ((via_pcr & 0x0e) == 0x0c) 
				{
					/* ca2 is outputting low */
					via_ca2 = 0;
					timerAddItem(via_ca2,&sig_zero, TIMER_ZERO);
				} 
				else 
				{
					/* ca2 is disabled or in pulse mode or is
					 * outputting high.
					 */
					via_ca2 = 1;
					timerAddItem(via_ca2,&sig_zero, TIMER_ZERO);
				}
				if ((via_acr & 0x1c) == 0)
				{
					if ((via_pcr & 0xe0) == 0xc0) 
					{
						/* cb2 is outputting low */
						via_cb2h = 0;
						timerAddItem(via_cb2h, &sig_blank, TIMER_BLANK_ON_CHANGE);
					} 
					else if ((via_pcr & 0xe0) == 0xe0) 
					{
						/* cb2 is outputting high */
						via_cb2h = 1;
						timerAddItem(via_cb2h, &sig_blank, TIMER_BLANK_OFF_CHANGE);
					} 
					else 
					{
						/* cb2 is disabled or is in pulse mode or is
						 * outputting high.
						 */
						via_cb2h = 1;
						timerAddItem(via_cb2h, &sig_blank, TIMER_BLANK_OFF_CHANGE);
					}
				}
				break;
            case 0xd:
				/* interrupt flag register */
				via_ifr &= ~(data & 0x7f);
				int_update ();
               break;
            case 0xe:
				/* interrupt enable register */
				if ((data & 0x80) !=0)
				{
					via_ier |= data & 0x7f;
				} 
				else 
				{
					via_ier &= ~(data & 0x7f);
				}
				int_update ();
               break;
         }
      }
   }

   
}



void vecx_reset (void)
{
	unsigned r;

	/* ram */

	for (r = 0; r < 1024; r++) {
		ram[r] = r & 0xff;
	}

	for (r = 0; r < 16; r++) {
		snd_regs[r] = 0;
		e8910_write(r, 0);
	}

	/* input buttons */

	snd_regs[14] = 0xff;
	e8910_write(14, 0xff);

	snd_select = 0;

	via_ora = 0;
	via_orb = 0;
	via_ddra = 0;
	via_ddrb = 0;
	via_t1on = 0;
	via_t1int = 0;
	via_t1c = 0;
	via_t1ll = 0;
	via_t1lh = 0;
	via_t1pb7 = 0x80;
	via_t2on = 0;
	via_t2int = 0;
	via_t2c = 0;
	via_t2ll = 0;
	via_sr = 0;
	via_srb = 8;
	via_src = 0;
	via_srclk = 0;
	via_acr = 0;
	via_pcr = 0;
	via_ifr = 0;
	via_ier = 0;
	old_via_ca1 = 1;
	via_ca1 = 1;
	via_ca2 = 1;
	via_cb2h = 1;
	via_cb2s = 0;

	alg_rsh = 128;
	alg_xsh = 128;
	alg_ysh = 128;
	alg_zsh = 0;
	alg_jch0 = 128;
	alg_jch1 = 128;
	alg_jch2 = 128;
	alg_jch3 = 128;
	alg_jsh = 128;

	alg_compare = 0; /* check this */

	alg_dx = 0;
	alg_dy = 0;
	alg_curr_x = ALG_MAX_X / 2;
	alg_curr_y = ALG_MAX_Y / 2;

	alg_vectoring = 0;


	alternate = 0; // 
	syncImpulse = 0;
	fcycles = FCYCLES_INIT;
	e6809_read8 = read8;
	e6809_write8 = write8;

	e6809_reset ();
	currentPB6 = 1;
	currentIRQ = 1;
	
	BANK_MAX = 1;
	initTimerArray();
	cyclesRunning = 0;


	e6809_reset ();
}


/* perform a single cycle worth of via emulation.
 * via_sstep0 is the first postion of the emulation.
 */

IRAM_ATTR static einline void via_sstep0 (void)
{
	int t2shift;
	if (via_t1on!=0) 
	{
		
		via_t1c--;
		if ((via_t1c & 0xffff) == 0xffff) // two cycle "MORE" since in via manual it says timer runs 1,5 cycles to long
		{
			/* counter just rolled over */
			if ((via_acr & 0x40) != 0)
			{
				/* continuous interrupt mode */
				via_ifr |= 0x40;
				int_update ();
				via_t1pb7 = 0x80 - via_t1pb7;
				doCheckRamp(0);
				/* reload counter */
				via_t1c = ((via_t1lh << 8)&0xff00) | (via_t1ll&0xff);
			} 
			else 
			{
				/* one shot mode */
				if (via_t1pb7 != 0x80)
				{
					via_t1pb7 = 0x80;
					doCheckRamp(0);
				}
				else
				{
					via_t1pb7 = 0x80;
				}
				if (via_t1int != 0) 
				{
					via_ifr |= 0x40;
					int_update ();
					via_t1int = 0;
				}
			}
		}
	}

	if ((via_t2on!=0) && (via_acr & 0x20) == 0x00) 
	{
		via_t2c--;
		if ((via_t2c & 0xffff) == 0xffff) // two cycle "MORE" since in via manual it says timer runs 1,5 cycles to long
		{
			/* one shot mode */
			if (via_t2int!=0) 
			{
				via_ifr |= 0x20;
				int_update ();
				via_t2int = 0;
				syncImpulse = 1;
			}
		}
	}

	// shift counter 
	via_src--;
	if ((via_src & 0xff) == 0xff) 
	{
		via_src = via_t2ll;
		if (via_srclk == 3) 
		{
			t2shift = 1;
			via_srclk = 0;
		} 
		else 
		{
			t2shift = 0;
			via_srclk = (via_srclk+1)%4;
		}
	} 
	else 
	{
		t2shift = 0;
	}
	if (via_srb < 8) 
	{
		switch (via_acr & 0x1c) 
		{
			case 0x00:
				// disabled 
				break;
			case 0x04:
				// shift in under control of t2 
				if (t2shift!=0) 
				{
					// shifting in 0s since cb2 is always an output 
					via_sr <<= 1;
					via_srb++;
				}
				break;
			case 0x08:
				// shift in under system clk control 
				via_sr <<= 1;
				via_srb++;
				break;
			case 0x0c:
				// shift in under cb1 control 
				break;
			case 0x10:
				// shift out under t2 control (free run) 
				if (t2shift!=0) 
				{
					via_cb2s = (via_sr >> 7) & 1;
					via_sr <<= 1;
					via_sr |= via_cb2s;

					timerAddItem(via_cb2s, &sig_blank, (via_cb2s==1) ? TIMER_BLANK_OFF_CHANGE : TIMER_BLANK_ON_CHANGE);
				}
				break;
			case 0x14:
				/// shift out under t2 control 
				if (t2shift!=0) 
				{
					via_cb2s = (via_sr >> 7) & 1;
					via_sr <<= 1;
					via_sr |= via_cb2s;
					timerAddItem(via_cb2s, &sig_blank, (via_cb2s==1) ? TIMER_BLANK_OFF_CHANGE : TIMER_BLANK_ON_CHANGE);
					via_srb++;
				}
				break;
			case 0x18:
/*                    
				// shift out under system clock control 
*/                    
				// System Time -> look at hardware manual
				// only every SECOND cycle!
				alternate = !alternate;
				if (alternate)
				{
					via_cb2s = (via_sr >> 7) & 1;
					via_sr <<= 1;
					via_sr |= via_cb2s;
					timerAddItem(via_cb2s, &sig_blank, (via_cb2s==1) ? TIMER_BLANK_OFF_CHANGE : TIMER_BLANK_ON_CHANGE);
					via_srb++;
				}
				break;
			case 0x1c:
				// shift out under cb1 control 
				break;
		}
		
		if (via_srb == 8)
		{
			via_ifr |= 0x04;
			int_update ();
			//lastShift = via_cb2s;
		}
	}
}


/* perform the second part of the via emulation */

IRAM_ATTR static einline void via_sstep1 (void)
{
	if ((via_pcr & 0x0e) == 0x0a) 
	{
		/* if ca2 is in pulse mode, then make sure
		 * it gets restored to '1' after the pulse.
		 */
		via_ca2 = 1;
		timerAddItem(via_ca2, &sig_zero, TIMER_ZERO);
	}

	if ((via_pcr & 0xe0) == 0xa0) 
	{
		/* if cb2 is in pulse mode, then make sure
		 * it gets restored to '1' after the pulse.
		 */
		via_cb2h = 1;
		timerAddItem(via_cb2h, &sig_blank, TIMER_BLANK_OFF_CHANGE);
	}

	// documentation of VIA
	if (via_ca1 !=old_via_ca1)
	{
		if ((via_pcr & 0x01) == 0x01) // interrupt flag is set by transition low to high
		{
			if (via_ca1 != 0)
			{
				via_ifr = via_ifr | 0x02;
				int_update();
			}
		}
		else // ((via_pcr & 0x01) == 0x00) // interrupt flag is set by transition high to low
		{
			if (via_ca1 == 0)
			{
				via_ifr = via_ifr | 0x02;
				int_update();
			}
		}
	}
	old_via_ca1 =via_ca1;// NEW

}

/* perform a single cycle worth of analog emulation */
IRAM_ATTR static einline void alg_sstep (void)
{
	long sig_dx=0, sig_dy=0;


	if (sig_zero == 0)
   {
      /* need to force the current point to the 'orgin' so just
       * calculate distance to origin and use that as dx,dy.
       */
      sig_dx = ALG_MAX_X/2  - alg_curr_x;
      sig_dy = ALG_MAX_Y/2  - alg_curr_y;
   }
   	
	
	
	if (sig_ramp== 0) 
	{
		sig_dx += alg_xsh;
		sig_dy += -alg_ysh;
	} 
	else 
	{
	}




   if (alg_vectoring == 0)
   {
      if ((sig_blank == 1 
		
		&& 
		alg_curr_x >= 0 && alg_curr_x < ALG_MAX_X && 
		alg_curr_y >= 0 && alg_curr_y < ALG_MAX_Y
		
	) 
		&& (((alg_zsh &0x80) ==0) &&  ((alg_zsh&0x7f) !=0) ) )
      {

         /* start a new vector */

         alg_vectoring = 1;
         alg_vector_x0 = alg_curr_x + DELAYS[TIMER_BLANK_OFF_CHANGE]*alg_xsh;
         alg_vector_y0 = alg_curr_y + DELAYS[TIMER_BLANK_OFF_CHANGE]*alg_ysh;
         alg_vector_x1 = alg_curr_x;
         alg_vector_y1 = alg_curr_y;
         alg_vector_dx = sig_dx;
         alg_vector_dy = sig_dy;
		 alg_ramping = (sig_ramp== 0);
         alg_vector_color = makeUnsigned((signed int)alg_zsh);
      }
   }
	
	
	else 
   {
      /* already drawing a vector ... check if we need to turn it off */

      if ((sig_blank == 0) || ((alg_zsh&0x80) !=0) || ((alg_zsh &0x7f) ==0))
      {
         /* blank just went on, vectoring turns off, and we've got a
          * new line.
          */

         alg_vectoring = 0;

		if (sig_blank == 0)
		{
			alg_addline (alg_vector_x0, 
						 alg_vector_y0, 
						 alg_vector_x1 + DELAYS[TIMER_BLANK_ON_CHANGE]*alg_xsh, 
						 alg_vector_y1 + DELAYS[TIMER_BLANK_ON_CHANGE]*alg_ysh, 
						 alg_vector_color);
		}
		else
			
		{
			alg_addline (alg_vector_x0, 
						 alg_vector_y0, 
						 alg_vector_x1, 
						 alg_vector_y1, 
						 alg_vector_color);
		}
      }
      else if (((sig_dx != alg_vector_dx) && (sig_ramp== 0)) || 
	           ((sig_dy != alg_vector_dy)&& (sig_ramp== 0)) || 
			   (makeUnsigned((signed int)alg_zsh) != alg_vector_color) || 
			   ((sig_ramp == 0) != alg_ramping)
			   )
      {
         /* the parameters of the vectoring processing has changed.
          * so end the current line.
          */
				alg_addline (alg_vector_x0, 
							 alg_vector_y0, 
							 alg_vector_x1, 
							 alg_vector_y1, 
							 alg_vector_color);


         /* we continue vectoring with a new set of parameters if the
          * current point is not out of limits.
          */

         if (alg_curr_x >= 0 && alg_curr_x < ALG_MAX_X &&
               alg_curr_y >= 0 && alg_curr_y < ALG_MAX_Y)
         {
            alg_vector_x0 = alg_curr_x;
            alg_vector_y0 = alg_curr_y;
            alg_vector_x1 = alg_curr_x;
            alg_vector_y1 = alg_curr_y;
			if (sig_ramp==0)
			{
				alg_vector_dx = alg_xsh;
				alg_vector_dy = -alg_ysh;
			}
			else
			{
				alg_vector_dx = 0;
				alg_vector_dy = 0;
			}
            alg_vector_color = makeUnsigned((signed int)alg_zsh);
         }
         else
            alg_vectoring = 0;
      }
	}
	

	alg_curr_x += sig_dx;
	alg_curr_y += sig_dy;

	if (alg_vectoring == 1 &&
		alg_curr_x >= 0 && alg_curr_x < ALG_MAX_X &&
		alg_curr_y >= 0 && alg_curr_y < ALG_MAX_Y) {

		/* we're vectoring ... current point is still within limits so
		 * extend the current vector.
		 */

		alg_vector_x1 = alg_curr_x;
		alg_vector_y1 = alg_curr_y;
	}
}
/*
IRAM_ATTR void vecxSteps (long icycles)
{
	for (int c = 0; c < icycles; c++) 
	{
		via_sstep0 ();
		timerDoStep();
		alg_sstep ();
		via_sstep1 ();
		e8910_tick ();
	}
}
*/
IRAM_ATTR void vecx_emu (long cycles)
{
	extern unsigned reg_pc;
	unsigned icycles;

	while (cycles > 0) 
	{
		icycles = e6809_sstep (via_ifr & 0x80, 0);
	    if (reg_pc == 0xf1a2) thisWaitRecal = 1;
		cyclesRunning += icycles;

		for (int c = 0; c < icycles; c++) {

			via_sstep0 ();
		    timerDoStep();
			alg_sstep ();
			via_sstep1 ();
			e8910_tick ();
			
		}

/*
// Handshake semaphores
extern SemaphoreHandle_t sem_job_available;   // Core0 waits on this
extern SemaphoreHandle_t sem_job_done;   // Core1 waits on this

// Data passed from Core1 -> Core0
extern volatile uint16_t core0_cycles;

        // 2. Dispatch previous instruction's cycles to Core 0, if we have one
        {
            // Wait until previous Core 0 job is done
            // (this will not block if Core 0 has already finished)
            xSemaphoreTake(sem_job_done, portMAX_DELAY);

            // Pass last_cycles to Core 0 and wake worker
            core0_cycles = icycles;
            xSemaphoreGive(sem_job_available);
            // Core 0 will now run the per-cycle loop for last_cycles
        }
*/










		cycles -= (long) icycles;
		fcycles -= (long) icycles;
		int doSync = 0;
		
		if (config_autoSync)
		{
			if (syncImpulse)
			{
				// some carts use T2 for other timing (like digital output), these timers are "realy" small compared to 50 Hz
				if (cyclesRunning - lastSyncCycles < 20000) // do not trust T2 timers which are to lo!
				{
					lastSyncCycles = cyclesRunning;
					syncImpulse = 0;
				}
			}
			if (syncImpulse)
			{
				// this check evens out some peaks above the 3000cycle range
				if (cyclesRunning - lastWaitRecal < 100000)
				{
					if (thisWaitRecal)
					{
						lastSyncCycles = cyclesRunning;
						doSync = 1;
					}
				}
				else
				{
					lastSyncCycles = cyclesRunning;
					doSync = 1;
				}
			}
			else if (fcycles < 0) 
			{
				doSync = 1;
			}
		}
		else
		{
			if (fcycles < 0) 
			{
				doSync = 1;
			}
		}
		
		if (doSync)
		{
			if (thisWaitRecal)
			{
				thisWaitRecal = 0;
				lastWaitRecal = cyclesRunning;
			}                

			syncImpulse = 0;
			fcycles = FCYCLES_INIT;
			emu_end_frame();
		}	  
	}
}