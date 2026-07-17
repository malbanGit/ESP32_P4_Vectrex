// NEW

//#define PROFILING 1

#include <stdio.h>
#include "e6809.h"
#include "e8910.h"
#include "vecx.h"
#include "system.h"

#include "esp_attr.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define einline __inline

extern char *cartName;
extern long cartSize;

#define MAX_CART_SIZE    32768 // for now! (256*1024)
extern DRAM_ATTR unsigned char cartData[MAX_CART_SIZE];



DRAM_ATTR static long xChange = 0;
DRAM_ATTR static long rChange = 0;

DRAM_ATTR static long cyclesRunning = 0;
DRAM_ATTR static int thisWaitRecal = 0;
DRAM_ATTR static long lastWaitRecal=0;
DRAM_ATTR static long lastSyncCycles = 0;
DRAM_ATTR static int syncImpulse = 0;

DRAM_ATTR static int config_autoSync = 1;
DRAM_ATTR static int sig_ramp = 0; 
DRAM_ATTR static int sig_blank = 0; // moved out of emu_loop for lightpen access
DRAM_ATTR static int sig_zero = 0;
DRAM_ATTR static uint8_t alternate = 0;
DRAM_ATTR static int currentBank = 0;
DRAM_ATTR static int currentIRQ = 1;
DRAM_ATTR static int currentPB6 = 1;
DRAM_ATTR static int pb6_in = 0x40; // 0 or 0x40 in from external
DRAM_ATTR static int pb6_out = 0x40; // out from vectrex
DRAM_ATTR static int BANK_MAX = 1;
//void timerAddItem(int value, void *destination, int t);

#define EMU_TIMER 20 /* the emulators heart beats at 20 milliseconds */
int SCREEN_WIDTH=800;
int SCREEN_HEIGHT=1280;

DRAM_ATTR static long scl_factorx;
DRAM_ATTR static long scl_factory;
DRAM_ATTR static long offx;
DRAM_ATTR static long offy;


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
DRAM_ATTR static uint16_t via_t1c;
DRAM_ATTR static uint8_t  via_t1ll;
DRAM_ATTR static uint8_t  via_t1lh;
DRAM_ATTR static unsigned via_t1pb7; /* timer 1 controlled version of pb7 */
DRAM_ATTR static unsigned via_t2on;  /* is timer 2 on? */
DRAM_ATTR static unsigned via_t2int; /* are timer 2 interrupts allowed? */
DRAM_ATTR static uint16_t via_t2c;
DRAM_ATTR static uint8_t via_t2ll;
DRAM_ATTR static uint8_t via_sr;
DRAM_ATTR static uint8_t via_srb;   /* number of bits shifted so far */
DRAM_ATTR static uint8_t via_src;   /* shift counter */
DRAM_ATTR static uint8_t via_srclk;
DRAM_ATTR static uint8_t via_acr;
DRAM_ATTR static uint8_t via_pcr;
DRAM_ATTR static uint8_t via_ifr;
DRAM_ATTR static uint8_t via_ier;
DRAM_ATTR static uint8_t old_via_ca1;
DRAM_ATTR static uint8_t via_ca1;
DRAM_ATTR static uint8_t via_ca2;
DRAM_ATTR static uint8_t via_cb2h;  /* basic handshake version of cb2 */
DRAM_ATTR static uint8_t via_cb2s;  /* version of cb2 controlled by the shift register */

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

	FCYCLES_INIT    = 30000,//VECTREX_MHZ / VECTREX_PDECAY,

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


#include "e8910.c"


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
DRAM_ATTR TimerItem timerItemArray[MAX_TIMER]; // should be saved

// Doppelt verketteter Listen-Knoten
typedef struct TimerNode_s {
    struct TimerNode_s *prev;
    struct TimerNode_s *next;
    TimerItem          *item;
} TimerNode;

// Knotenpuffer (nicht "persistent" nötig)
DRAM_ATTR static TimerNode timerNodeArray[MAX_TIMER];

// Listen-Köpfe
DRAM_ATTR static TimerNode *freeListHead = NULL;   // ungenutzte Items
DRAM_ATTR static TimerNode *usedListHead = NULL;   // genutzte Items


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
#define TIMER_BLANK_ON_CHANGE_VALUE_IS_NULL 1
	0,  // TIMER_BLANK_OFF_CHANGE = 3,
#define TIMER_BLANK_OFF_CHANGE_VALUE_IS_NULL 1

	12, // TIMER_RAMP_CHANGE = 4,
	14, // TIMER_MUX_Y_CHANGE = 5,
#define TIMER_MUX_S_CHANGE_VALUE_IS_NULL 1	
	0,  // TIMER_MUX_S_CHANGE = 6,
#define TIMER_MUX_Z_CHANGE_VALUE_IS_NULL 1	
	0,  // TIMER_MUX_Z_CHANGE = 7,
#define TIMER_MUX_R_CHANGE_VALUE_IS_NULL 1
	0,  // TIMER_MUX_R_CHANGE = 8,
	12, // TIMER_XSH_CHANGE = 9,					// if this gets too high - then line buffer will explode, since a "fake" middline change is registered, and lines are splitted
#define TIMER_LIGHTPEN_VALUE_IS_NULL 1
	0,  // TIMER_LIGHTPEN = 10,
	15, // TIMER_RAMP_OFF_CHANGE = 11,
	1,  // TIMER_MUX_SEL_CHANGE = 12, 
#define TIMER_SHIFT_VALUE_IS_NULL 1	
	0,  // TIMER_SHIFT = 13, 
	
#define TIMER_T1_VALUE_IS_NULL	1
	0,  // TIMER_T1 = 14, 
#define TIMER_T2_VALUE_IS_NULL	1
	0   // TIMER_T2 = 15,
	}; // no need to be saved
DRAM_ATTR int rampOnFractionValue = -10;
DRAM_ATTR int rampOffFractionValue = -15;
DRAM_ATTR int blankOnDelay = -2; // Karl -2
DRAM_ATTR int blankOffDelay = -2;
DRAM_ATTR bool rampOnFraction = false;
DRAM_ATTR bool rampOffFraction = false;


DRAM_ATTR static long fcycles;


void emu_begin_frame(void);
void emu_end_frame(void);
void emu_draw_line(int x0, int y0, int x1, int y1, uint8_t brightness); // For ESP32
void vecx_emu (long cycles);
#include "usb/hid_usage_keyboard.h"


IRAM_ATTR static einline void readevents()
{
	// Public API:
	extern IRAM_ATTR bool isKeyDown(uint8_t hid_code);
	extern IRAM_ATTR bool isAsciiDown(char c);
	extern IRAM_ATTR bool isShiftDown(void);
	extern IRAM_ATTR bool isCtrlDown(void);
	extern IRAM_ATTR bool isAltDown(void);

	#ifndef HID_KEY_LEFT_ARROW
	#define HID_KEY_LEFT_ARROW   0x50
	#endif

	#ifndef HID_KEY_RIGHT_ARROW
	#define HID_KEY_RIGHT_ARROW  0x4F
	#endif

	#ifndef HID_KEY_UP_ARROW
	#define HID_KEY_UP_ARROW     0x52
	#endif

	#ifndef HID_KEY_DOWN_ARROW
	#define HID_KEY_DOWN_ARROW   0x51
	#endif

// center is default unless pressed!
alg_jch0=127;
alg_jch1=127;
alg_jch2=127;
alg_jch3=127;
   /* Player 1 */
 	if (isKeyDown(HID_KEY_LEFT_ARROW)) alg_jch0 = 0;
 	if (isKeyDown(HID_KEY_RIGHT_ARROW)) alg_jch0 = 255;
 	if (isKeyDown(HID_KEY_UP_ARROW)) alg_jch1 = 255;
 	if (isKeyDown(HID_KEY_DOWN_ARROW)) alg_jch1 = 0;

extern  int  g_line_width;      
extern  int  g_line_glow;
extern  int brightnessAdjust;

long load_rom_file(const char *rom_name, uint8_t *buffer, long max_size);
static int loadNum = 0;
int vecx_init();
char  *name[]={
	"SWEEP.BIN", 
	"KARL.BIN",
	"TGARD.BIN",
	"VBLADE.NIB",
	"ARMOR.BIN",
	"BERZERK.BIN",
	"RELEASE.BIN",
	"SPIKE.BIN"
};

   	if (isAsciiDown('o'))
	{
		loadNum--;
		if (loadNum<0) loadNum = 7;
        cartSize = load_rom_file(name[loadNum], cartData, sizeof(cartData));
		vecx_init();
	}
   	if (isAsciiDown('p'))
	{
		loadNum++;
		if (loadNum>7) loadNum = 0;
        cartSize = load_rom_file(name[loadNum], cartData, sizeof(cartData));
		vecx_init();
	}
	if (isKeyDown(HID_KEY_F11))
	{
		brightnessAdjust = brightnessAdjust - 1;
		printf("brightnessAdjust: %d\n", brightnessAdjust);
	}
	if (isKeyDown(HID_KEY_F12))
	{
		brightnessAdjust = brightnessAdjust + 1;
		printf("brightnessAdjust: %d\n", brightnessAdjust);
	}
   
	if (isKeyDown(HID_KEY_F1))
	{
		g_line_width = g_line_width - 1;
		if (g_line_width<0) g_line_width = 0;
		printf("g_line_width: %d\n", g_line_width);
	}
	if (isKeyDown(HID_KEY_F2))
	{
		g_line_width = g_line_width + 1;
		printf("g_line_width: %d\n", g_line_width);
	}
	if (isKeyDown(HID_KEY_F3))
	{
		g_line_glow = g_line_glow - 1;
		if (g_line_glow<0) g_line_glow = 0;
		printf("g_line_glow: %d\n", g_line_glow);
	}
	if (isKeyDown(HID_KEY_F4))
	{
		g_line_glow = g_line_glow + 1;
		printf("g_line_glow: %d\n", g_line_glow);
	}
	
	if (isKeyDown(HID_KEY_1))
	{
		blankOnDelay = blankOnDelay - 1;
		printf("blankOnDelay: %d\n", blankOnDelay);
	}
	if (isKeyDown(HID_KEY_2))
	{
		blankOnDelay = blankOnDelay + 1;
		printf("blankOnDelay: %d\n", blankOnDelay);
	}

	if (isKeyDown(HID_KEY_3))
	{
		blankOffDelay = blankOffDelay - 1;
		printf("blankOffDelay: %d\n", blankOffDelay);
	}
	if (isKeyDown(HID_KEY_4))
	{
		blankOffDelay = blankOffDelay + 1;
		printf("blankOffDelay: %d\n", blankOffDelay);
	}
	if (isKeyDown(HID_KEY_5))
	{
		rampOnFractionValue = rampOnFractionValue - 1;
		printf("rampOnFractionValue: %d\n", rampOnFractionValue);
	}
	if (isKeyDown(HID_KEY_6))
	{
		rampOnFractionValue = rampOnFractionValue + 1;
		printf("rampOnFractionValue: %d\n", rampOnFractionValue);
	}
	if (isKeyDown(HID_KEY_7))
	{
		rampOffFractionValue = rampOffFractionValue - 1;
		printf("rampOffFractionValue: %d\n", rampOffFractionValue);
	}
	if (isKeyDown(HID_KEY_8))
	{
		rampOffFractionValue = rampOffFractionValue + 1;
		printf("rampOffFractionValue: %d\n", rampOffFractionValue);
	}



   if (isAsciiDown('a'))
      snd_regs[14] &= ~1;
   else
      snd_regs[14] |= 1;

   if (isAsciiDown('s'))
      snd_regs[14] &= ~2;
   else
      snd_regs[14] |= 2;

   if (isAsciiDown('d'))
      snd_regs[14] &= ~4;
   else
      snd_regs[14] |= 4;

   if (isAsciiDown('f'))
      snd_regs[14] &= ~8;
   else
      snd_regs[14] |= 8;

   /* Player 2 */
 	if (isKeyDown('h')) alg_jch2 = 0; // left
 	if (isKeyDown('j')) alg_jch2 = 255;
 	if (isKeyDown('u')) alg_jch3 = 255; // up
 	if (isKeyDown('n')) alg_jch3 = 0;
	
   if (isAsciiDown('q'))
      snd_regs[14] &= ~16;
   else
      snd_regs[14] |= 16;

   if (isAsciiDown('w'))
      snd_regs[14] &= ~32;
   else
      snd_regs[14] |= 32;

   if (isAsciiDown('e'))
      snd_regs[14] &= ~64;
   else
      snd_regs[14] |= 64;

   if (isAsciiDown('r'))
      snd_regs[14] &= ~128;
   else
      snd_regs[14] |= 128;
}

IRAM_ATTR void osint_emuloop(int cycles)
{
	vecx_emu(cycles);
	readevents();
}

// resize and center to given width / height
void resize(int width, int height){

	scl_factorx = ALG_MAX_X / width;
	scl_factory = ALG_MAX_Y / height;


	offx = (width - ALG_MAX_X / scl_factorx) / 2;
	offy = (height - ALG_MAX_Y / scl_factory) / 2;

	offx = (  ALG_MAX_X / scl_factorx) / 2;
	offy = (  ALG_MAX_Y / scl_factory) / 2;
	offx = (SCREEN_WIDTH-width)/2;
	offy = (SCREEN_HEIGHT-height)/2;
}

int vecx_init()
{
	if (cartSize != 0)
	{
		//cartData
		// a cartridge was loaded!
	}
	
//	resize(SCREEN_HEIGHT*3/4, SCREEN_HEIGHT); // this is easily to much for high resolutions
	resize(500, 700);

	vecx_reset();
	e8910_init_sound();
//	e8910_done_sound();
	return 0;
}

IRAM_ATTR static einline  void alg_addline (long x0, long y0, long x1, long y1, unsigned char color)
{
	emu_draw_line(offx + x0 / scl_factorx, offy +y0 / scl_factory, offx + x1 / scl_factorx, offy + y1 / scl_factory, color); // For ESP32
//	emu_draw_line(offx + x0 / scl_factorx, offy -y0 / scl_factory, offx + x1 / scl_factorx, offy - y1 / scl_factory, color); // For ESP32
	return;
}

static IRAM_ATTR einline void detach_from_list(TimerNode **head, TimerNode *node)
{
    if (node == NULL) return;

    if (node->prev != NULL) {
        node->prev->next = node->next;
    } else {
        // node war Kopf
        *head = node->next;
    }

    if (node->next != NULL) {
        node->next->prev = node->prev;
    }

    node->prev = NULL;
    node->next = NULL;
}

// Knoten vorne an eine Liste anhängen
static IRAM_ATTR einline  void push_front(TimerNode **head, TimerNode *node)
{
    if (node == NULL) return;

    node->prev = NULL;
    node->next = *head;

    if (*head != NULL) {
        (*head)->prev = node;
    }

    *head = node;
}

IRAM_ATTR
static inline __attribute__((always_inline, hot)) void timerAddItem(int value, void *destination, int ty)
{

    if (freeListHead == NULL) {
		printf("NOT ENOUGH TIMERS! PANIC!!!\n");
		exit(3);
    }

 // Kopf von freeList nehmen
    TimerNode *node = freeListHead;
    detach_from_list(&freeListHead, node);

    TimerItem *t = node->item;

    t->countDown  = DELAYS[(ty&0xff)];
    t->valueToSet = value&0xff;
    t->whereToSet = destination;
    t->type       = ty;
    t->active     = 1;

    // vorne in usedList einfügen
    push_front(&usedListHead, node);
}

IRAM_ATTR
static inline __attribute__((always_inline, hot)) void doCheckRamp(int fromOrbWrite)
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
IRAM_ATTR
static inline __attribute__((always_inline, hot)) signed int makeSigned(unsigned char data)
{
   if (data > 127) return -(256-data);
   return data;
}

IRAM_ATTR
static inline __attribute__((always_inline, hot)) unsigned char makeUnsigned(signed int data)
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

IRAM_ATTR
static inline __attribute__((always_inline, hot)) void int_update ()
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
IRAM_ATTR
static inline __attribute__((always_inline, hot)) void doCheckMultiplexer()
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
#ifdef TIMER_MUX_R_CHANGE_VALUE_IS_NULL	
			alg_rsh = makeSigned(alg_xsh);
#else
			timerAddItem(alg_DAC, 0, TIMER_MUX_R_CHANGE);
#endif

			break;
		case 0x04:
			/* demultiplexor is on */
#ifdef TIMER_MUX_Z_CHANGE_VALUE_IS_NULL	
			alg_zsh = alg_DAC;
#else
			timerAddItem(alg_DAC , &alg_zsh, TIMER_MUX_Z_CHANGE);
#endif

// todo
			//			intensityDrift = 0;
			break;
		case 0x06:
			/* sound output line */
#ifdef TIMER_MUX_S_CHANGE_VALUE_IS_NULL	
			alg_ssh = makeSigned(alg_DAC);
#else
			timerAddItem(alg_DAC , &alg_ssh, TIMER_MUX_S_CHANGE);
#endif

			break;
			
	}
}

void TimerList_init(void)
{
    freeListHead = NULL;
    usedListHead = NULL;

    for (int i = 0; i < MAX_TIMER; i++) 
	{
        timerItemArray[i].active     = 0;
        timerItemArray[i].countDown  = 0;
        timerItemArray[i].valueToSet = 0;
        timerItemArray[i].whereToSet = NULL;
        timerItemArray[i].type       = TIMER_ACTION_NONE;

  // Knoten verbinden
        TimerNode *node = &timerNodeArray[i];
        node->item = &timerItemArray[i];

        // vorne an freeList einfügen
        node->prev = NULL;
        node->next = freeListHead;

        if (freeListHead != NULL) {
            freeListHead->prev = node;
        }
        freeListHead = node;
    }
}


static IRAM_ATTR einline void timerDoStep()
{
	int doInt = 0;
	int doMUL = 0;
	int doRamp = 0;
	TimerNode *node = usedListHead;


	while (node != NULL)
	{
		TimerItem *item = node->item;

		item->countDown--;
		if (item->countDown <=0)
		{
#ifdef TIMER_SHIFT_VALUE_IS_NULL	
#else
			if (item->type == TIMER_SHIFT_READ)
			{
				alternate = 1;
				//lastShiftTriggered = cyclesRunning;
				via_ifr &= 0xfb; /* remove shift register interrupt flag */
				via_srb = 0;
				via_srclk = 1;
				doInt++;
			} 
			else if (item->type == TIMER_SHIFT_WRITE)
			{
				alternate = 1;
				//via_stalling = 0;
				//lastShiftTriggered = cyclesRunning;
				via_sr = item->valueToSet;
				via_ifr &= 0xfb; /* remove shift register interrupt flag */
				via_srb = 0;
				via_srclk = 1;
				doInt++;
			}
			else 
#endif
#ifdef TIMER_T1_VALUE_IS_NULL	
#else
			if (item->type == TIMER_T1)
			{
				via_t1on = 1; /* timer 1 starts running */
				via_t1lh = item->valueToSet;
				via_t1c = (via_t1lh << 8) | via_t1ll;
				via_ifr &= 0xbf; /* remove timer 1 interrupt flag */
				via_t1int = 1;

				via_t1pb7 = 0;
				doRamp++;
				doInt++;
			}
			else 
#endif
#ifdef TIMER_T2_VALUE_IS_NULL	
#else
			if (item->type == TIMER_T2)
			{
				via_t2c = ((item->valueToSet) << 8) | via_t2ll;
				via_t2c += 0; // hack, it seems vectrex (via) takes two cycles to "process" the setting...
				via_ifr &= 0xdf;
				via_t2on = 1; /* timer 2 starts running */
				via_t2int = 1;
				doInt++;
			}
			else 
#endif			
#ifdef TIMER_MUX_R_CHANGE_VALUE_IS_NULL	
#else
			if (item->type == TIMER_MUX_R_CHANGE)
			{
				//noiseCycles = cyclesRunning;
				// setDigitalVoltage(timerItemArray[i].valueToSet); 
				alg_rsh = alg_xsh;
			}
			else 
#endif
			
			if (item->whereToSet != 0)   
			{
				if (item->type == TIMER_RAMP_OFF_CHANGE) 
				{
					// difference of 3 is to much, but we have no "smaller" unit than
					// cycles ticks
					// therefor we calculate a fraction on ticks to be as exact as possible
					// the fraction here is not KNOWN, it is
					// experimented
					// analog curcuits don't really care about cycles...
					if ((*item->whereToSet & 0xff) != (item->valueToSet & 0xff))
					{
						rampOffFraction = 1;
					}
					rChange = cyclesRunning;

				}
				else if (item->type == TIMER_RAMP_CHANGE) 
				{
					// difference of 3 is to much, but we have no "smaller" unit than
					// cycles ticks
					// therefor we calculate a fraction on ticks to be as exact as possible
					// the fraction here is not KNOWN, it is
					// experimented
					// analog curcuits don't really care about cycles...
					if ((*item->whereToSet & 0xff) != (item->valueToSet & 0xff))
					{
						rampOnFraction = 1;
					}
					rChange = cyclesRunning;
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
				if (item->type == TIMER_MUX_Z_CHANGE) // Z must not be negative
				{
					*item->whereToSet = makeUnsigned(item->valueToSet & 0xff);
				}
				else
				{
					*item->whereToSet = makeSigned(item->valueToSet & 0xff);
					if (item->type == TIMER_XSH_CHANGE)
						xChange = cyclesRunning;
				}
				if (item->type == TIMER_MUX_SEL_CHANGE)
					doMUL++;
#ifdef TIMER_MUX_Z_CHANGE_VALUE_IS_NULL	
#else
#endif

			
			}

			item->active = 0;
			TimerNode *n = node->next;
			detach_from_list(&usedListHead, node);
			push_front(&freeListHead, node);
			node = n;
			continue;

		}
		node = node->next;
	}
	if (doRamp>0) doCheckRamp(0);
	if (doInt>0) int_update ();
	if (doMUL>0) doCheckMultiplexer();
}


/* update the snd chips internal registers when via_ora/via_orb changes */
IRAM_ATTR
static inline __attribute__((always_inline, hot)) void snd_update(int command)
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

#ifdef TIMER_SHIFT_VALUE_IS_NULL	
				alternate = 1;
				//lastShiftTriggered = cyclesRunning;
				via_ifr &= 0xfb; /* remove shift register interrupt flag */
				via_srb = 0;
				via_srclk = 1;
				int_update ();
#else
               timerAddItem(via_sr, 0, TIMER_SHIFT_READ);
#endif
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
	   if (BANK_MAX<4) 
		   data = cartData[address+(currentBank *32768)] & 0xff; // 
	   else 
		   data = cartData[address+(currentBank *65536)] & 0xff; // 
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
#ifdef TIMER_BLANK_ON_CHANGE_VALUE_IS_NULL
sig_blank = 0;
#else
timerAddItem(via_cb2h, &sig_blank, TIMER_BLANK_ON_CHANGE);
#endif


	
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


#ifdef TIMER_T1_VALUE_IS_NULL	
				via_t1on = 1; /* timer 1 starts running */
				via_t1lh = data;
				via_t1c = (via_t1lh << 8) | via_t1ll;
				via_ifr &= 0xbf; /* remove timer 1 interrupt flag */
				via_t1int = 1;

				via_t1pb7 = 0;
				doCheckRamp(0);
				int_update ();
#else
               timerAddItem(data,0, TIMER_T1);
#endif


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

#ifdef TIMER_T2_VALUE_IS_NULL	
				via_t2c = ((data) << 8) | via_t2ll;
				via_t2c += 0; // hack, it seems vectrex (via) takes two cycles to "process" the setting...
				via_ifr &= 0xdf;
				via_t2on = 1; /* timer 2 starts running */
				via_t2int = 1;
				int_update ();
#else
               timerAddItem(data,0, TIMER_T2);
#endif


               break;
            case 0xa:
#ifdef TIMER_SHIFT_VALUE_IS_NULL	
				alternate = 1;
				//via_stalling = 0;
				//lastShiftTriggered = cyclesRunning;
				via_sr = data;
				via_ifr &= 0xfb; /* remove shift register interrupt flag */
				via_srb = 0;
				via_srclk = 1;
				int_update ();
#else
				timerAddItem(data, &via_sr, TIMER_SHIFT_WRITE);
#endif
               break;
            case 0xb:
				if ((via_acr & 0x1c) != (data & 0x1c))
				{
					if ((data & 0x1c) == 0) // shift reg is switched off - so take the manual value
					{
					}
					else // use the last shift
					{
#ifdef TIMER_BLANK_ON_CHANGE_VALUE_IS_NULL
sig_blank = 0;
#else
timerAddItem(0, &sig_blank, TIMER_BLANK_ON_CHANGE);
#endif

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
#ifdef TIMER_BLANK_ON_CHANGE_VALUE_IS_NULL
sig_blank = 0;
#else
timerAddItem(0, &sig_blank, TIMER_BLANK_ON_CHANGE);
#endif
					} 
					else if ((via_pcr & 0xe0) == 0xe0) 
					{
						/* cb2 is outputting high */
						via_cb2h = 1;
#ifdef TIMER_BLANK_OFF_CHANGE_VALUE_IS_NULL
sig_blank = 1;
#else
timerAddItem(1, &sig_blank, TIMER_BLANK_OFF_CHANGE);
#endif
					} 
					else 
					{
						/* cb2 is disabled or is in pulse mode or is
						 * outputting high.
						 */
						via_cb2h = 1;
#ifdef TIMER_BLANK_OFF_CHANGE_VALUE_IS_NULL
sig_blank = 1;
#else
timerAddItem(1, &sig_blank, TIMER_BLANK_OFF_CHANGE);
#endif
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


	BANK_MAX = 1;
	{
	  currentBank = 0; // 
	  if (cartSize > 50000)
	  {
		BANK_MAX = 2;
		currentBank = 1; // 
	  }
	  if (cartSize > 100000)
	  {
		BANK_MAX = 4;
		currentBank = 3; // 
	  }
	}
	TimerList_init();
	cyclesRunning = 0;

	e6809_reset ();
}


/* perform a single cycle worth of via emulation.
 * via_sstep0 is the first postion of the emulation.
 */

IRAM_ATTR
static inline __attribute__((always_inline, hot)) void via_sstep0(void)
{
    int t2shift;
//    uint8_t acr = via_acr;   // einmal lesen, fürs Maskieren verwenden

    /* --- T1-Timer --- */
	if (via_t1on) 
	{
		via_t1c--;
		if (via_t1c == 0xFFFFu)
		{
			if (via_acr & 0x40u)
			{
				via_ifr |= 0x40u;
				int_update();
				via_t1pb7 = 0x80u - via_t1pb7;
				doCheckRamp(0);
				via_t1c = ((uint16_t)via_t1lh << 8) | via_t1ll;
			}
			else
			{
				if (via_t1pb7 != 0x80u)
				{
					via_t1pb7 = 0x80u;
					doCheckRamp(0);
				}
				else
				{
					via_t1pb7 = 0x80u;
				}

				if (via_t1int)
				{
					via_ifr |= 0x40u;
					int_update();
					via_t1int = 0;
				}
			}
		}
	}
	
    /* --- T2-Timer (nur wenn nicht Shift-Clock-Modus) --- */
    if (via_t2on && ((via_acr & 0x20u) == 0u)) 
    {
        via_t2c--;

        if ((uint16_t)via_t2c == 0xFFFFu)   // "läuft 1,5 Takte zu lang"
        {
            /* one shot mode */
            if (via_t2int) 
            {
                via_ifr |= 0x20;
                int_update();
                via_t2int = 0;
                syncImpulse = 1;
            }
        }
    }







	/* Gemeinsame Shift-Out-Sequenz */
	#ifdef TIMER_BLANK_ON_CHANGE_VALUE_IS_NULL
	#define DO_SHIFT_OUT()                           \
		do {                                         \
			via_cb2s = (via_sr >> 7) & 1u;           \
			via_sr <<= 1;                            \
			via_sr |= via_cb2s;                      \
			sig_blank = via_cb2s;                    \
		} while (0)
	#else
	#define DO_SHIFT_OUT()                           \
		do {                                         \
			via_cb2s = (via_sr >> 7) & 1u;           \
			via_sr <<= 1;                            \
			via_sr |= via_cb2s;                      \
			timerAddItem(                            \
				via_cb2s,                            \
				&sig_blank,                          \
				(via_cb2s == 1u) ? TIMER_BLANK_OFF_CHANGE \
								: TIMER_BLANK_ON_CHANGE  \
			);                                       \
		} while (0)
	#endif

	/* --- Shift-Teil von via_sstep0 --- */

	if (via_srb >= 8u)
		return;    /* oder zum Ende von via_sstep0 springen */


	/* Shift-Counter: exakt wie vorher, nur enger geschrieben */
	via_src--;
	if (via_src == 0xFFu)
	{
		via_src = via_t2ll;

		if (via_srclk == 3u) {
			t2shift  = 1;
			via_srclk = 0u;
		} else {
			t2shift  = 0;
			via_srclk = (uint8_t)((via_srclk + 1u) & 3u);  /* schneller als %4 */
		}
	}
	else
	{
		t2shift = 0;
	}

	/* Shift-Register nur solange noch Bits zu schieben sind */
	if (via_srb < 8u)
	{
		uint8_t mode = via_acr & 0x1Cu;

		switch (mode)
		{
			case 0x18:
				/* shift out unter System-Clock, jeder zweite Zyklus */
				alternate = !alternate;
				if (likely(alternate)) {
					DO_SHIFT_OUT();
					via_srb++;
				}
				break;

			case 0x10:
				/* shift out unter T2-Kontrolle (free run) */
				if (t2shift) {
					DO_SHIFT_OUT();
				}
				break;

			case 0x14:
				/* shift out unter T2-Kontrolle */
				if (t2shift) {
					DO_SHIFT_OUT();
					via_srb++;
				}
				break;

			case 0x1C:
				/* shift out unter CB1-Kontrolle */
				break;

			case 0x00:
				/* disabled */
				break;

			case 0x04:
				/* shift in unter Kontrolle von T2 */
				if (t2shift) {
					/* 0-en reinschieben (CB2 ist immer Output) */
					via_sr <<= 1;
					via_srb++;
				}
				break;

			case 0x08:
				/* shift in unter System-Clock */
				via_sr <<= 1;
				via_srb++;
				break;

			case 0x0C:
				/* shift in unter CB1-Kontrolle */
				break;

		}

		if (via_srb == 8u)
		{
			via_ifr |= 0x04u;
			int_update();
			/* lastShift = via_cb2s;  // falls du das später wieder brauchst */
		}
	}











	

}


/* perform the second part of the via emulation */
IRAM_ATTR
static inline __attribute__((always_inline, hot)) void via_sstep1(void)
{

    /* CA2 pulse mode: restore to '1' after pulse */
    if ((via_pcr & 0x0Eu) == 0x0Au)
    {
        via_ca2 = 1;
        timerAddItem(via_ca2, &sig_zero, TIMER_ZERO);
    }

    /* CB2 pulse mode: restore to '1' after pulse */
    if ((via_pcr & 0xE0u) == 0xA0u)
    {
        via_cb2h = 1;
    #ifdef TIMER_BLANK_OFF_CHANGE_VALUE_IS_NULL
        sig_blank = 1;
    #else
        timerAddItem(1, &sig_blank, TIMER_BLANK_OFF_CHANGE);
    #endif
    }

    /* CA1 edge detection / interrupt */
    if (via_ca1 != old_via_ca1)
    {
        // via_pcr bit0: 1 = low->high triggers, 0 = high->low triggers
        uint8_t trigger_on_rising = (via_pcr & 0x01u) != 0u;

        if (trigger_on_rising ? (via_ca1 != 0u) : (via_ca1 == 0u))
        {
            via_ifr |= 0x02u;
            int_update();
        }
    }

    // Remember last CA1 state
    old_via_ca1 = via_ca1;
}

IRAM_ATTR
static inline __attribute__((always_inline, hot)) void _old (void)
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
#ifdef TIMER_BLANK_OFF_CHANGE_VALUE_IS_NULL
sig_blank = 1;
#else
timerAddItem(1, &sig_blank, TIMER_BLANK_OFF_CHANGE);
#endif
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

IRAM_ATTR
static inline __attribute__((always_inline, hot)) void alg_sstep (void)
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
		if (rampOnFraction)
		{
			rampOnFraction = false;
			alg_curr_x -= (makeSigned(alg_xsh)*(rampOnFractionValue))/10;
			alg_curr_y -= - (makeSigned(alg_ysh)*(rampOnFractionValue))/10;

		}
	} 
	else 
	{
		if (rampOffFraction)
		{
			rampOffFraction = false;
			alg_curr_x += (makeSigned(alg_xsh)*(rampOffFractionValue))/10;
			alg_curr_y += - (makeSigned(alg_ysh)*(rampOffFractionValue))/10;
                if (alg_vectoring == 1 && alg_curr_x >= 0 && alg_curr_x < ALG_MAX_X && alg_curr_y >= 0 && alg_curr_y < ALG_MAX_Y) 
                {
                    /* we're vectoring ... current point is still within limits so
                     * extend the current vector.
                     */
                    alg_vector_x1 = (int)alg_curr_x;
                    alg_vector_y1 = (int)alg_curr_y;
                }  
		}
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

		long adderX = (long)((blankOffDelay*makeSigned(alg_xsh))/10);
		long adderY = (long)((blankOffDelay*makeSigned(alg_ysh))/10);


		 alg_vector_x0 = alg_curr_x + adderX;
         alg_vector_y0 = alg_curr_y + adderY;

		 alg_vector_x1 = alg_curr_x;
         alg_vector_y1 = alg_curr_y;
		 alg_vector_dx = alg_xsh;
		 alg_vector_dy = -alg_ysh;
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


		// Jul 26
			long adderX = (long)((blankOnDelay*makeSigned(alg_xsh))/10);
			long adderY = (long)((blankOnDelay*makeSigned(alg_ysh))/10);

			alg_addline (alg_vector_x0, 
						 alg_vector_y0, 
						 alg_vector_x1 + adderX, 
						 alg_vector_y1 + adderY, 
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

//was muss ich nehmen für KQ???



      else if (((alg_xsh != alg_vector_dx) && (sig_ramp== 0) ) || 
	           ((-alg_ysh != alg_vector_dy)&& (sig_ramp== 0)) || 
			   (makeUnsigned((signed int)alg_zsh) != alg_vector_color)  
// removed Dec 2025		||	   ((sig_ramp == 0) != alg_ramping)
			   )
			   
      {

// for splines and curved vectors the following is relevant
// as it is - for now in this emulator it is not working correctly :-()

		if ((cyclesRunning - rChange) == 0)
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
			goto contx;
		}

         /* the parameters of the vectoring processing has changed.
          * so end the current line.
          */
				alg_addline (alg_vector_x0, 
							 alg_vector_y0, 
							 alg_vector_x1, 
							 alg_vector_y1, 
							 alg_vector_color);
/*
							 printf("-changed-");

		if (alg_xsh!=alg_vector_dx) printf("(alg_xsh(%d)!=alg_vector_dx(%d)- (%ld), xChange=%ld, rChange=%ld\n",alg_xsh,(int)alg_vector_dx, cyclesRunning, (cyclesRunning - xChange), (cyclesRunning - rChange));
		if (-alg_ysh!=alg_vector_dy) printf("(alg_ysh(%d)!=alg_vector_dy(%d)\n", (-alg_ysh), (int)alg_vector_dy);
		if (makeUnsigned((signed int)alg_zsh) != alg_vector_color) 
			printf("(makeUnsigned((signed int)alg_zsh)(%d) != alg_vector_color(%d))\n", makeUnsigned((signed int)alg_zsh), alg_vector_color );
//		if ((sig_ramp == 0) != alg_ramping) printf("((sig_ramp == 0) != alg_ramping)\n");

*/

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
contx:			
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

#ifdef PROFILING
uint64_t d0 = 0;
uint64_t d1 = 0;
uint64_t d2 = 0;
uint64_t d3 = 0;
uint64_t d4 = 0;

uint64_t d0s = 0;
uint64_t d1s = 0;
uint64_t d2s = 0;
uint64_t d3s = 0;
uint64_t d4s = 0;
#endif
	while (cycles > 0) 
	{
#ifdef PROFILING
uint64_t m0 = esp_timer_get_time();
#endif

		icycles = e6809_sstep (via_ifr & 0x80, 0);
	    if (reg_pc == 0xf1a2) thisWaitRecal = 1;

#ifdef PROFILING
uint64_t m1 = esp_timer_get_time();
d0 = m1-m0;
d1 = 0;d2 = 0;d3 = 0;d4 = 0;
#endif

		for (int c = 0; c < icycles; c++) {
			cyclesRunning++;
			via_sstep0 ();
#ifdef PROFILING
uint64_t m2 = esp_timer_get_time();
			d1 += (m2-m1);
#endif
		    timerDoStep();
#ifdef PROFILING
uint64_t m3 = esp_timer_get_time();
			d2 += (m3-m2);
#endif
			alg_sstep ();
#ifdef PROFILING
uint64_t m4 = esp_timer_get_time();
			d3 += (m4-m3);
#endif
			via_sstep1 ();
#ifdef PROFILING
uint64_t m5 = esp_timer_get_time();
			d4 += (m5-m4);
#endif

#ifdef PROFILING
		 m1 = esp_timer_get_time();
#endif
		}

#ifdef PROFILING
d0s += d0;
d1s += d1;
d2s += d2;
d3s += d3;
d4s += d4;
#endif


		cycles -= (long) icycles;
		fcycles -= (long) icycles;
		int doSync = 0;
		
config_autoSync=1;		
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
				//doSync = 1;
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
#ifdef PROFILING
printf("6809 : %d\n", (int)d0s);
printf("via 0: %d\n", (int)d1s);		
printf("timer: %d\n", (int)d2s);		
printf("alg  : %d\n", (int)d3s);		
printf("via 1: %d\n", (int)d4s);		
#endif

}


