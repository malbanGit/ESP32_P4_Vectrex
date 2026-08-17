#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "esp_timer.h"

#include "esp_events.h"
#include "usb/hid_usage_keyboard.h"

//#define NO_AUDIO 1 just be quiet!
#define DEFAULT_AUTO_SYNC 1 // !! autosync only works reliably when WaitRecal is used. Spike speach out - does not use WR -> frames are missed!!!

#define VECX_DEBUG 1
#define MAX_EMU_FPS 50
#define ENABLE_OVERLAYS 1

#define LINE_WIDTH 1
#define LINE_GLOW_WIDTH 2
#define BRIGHTNESS_ADJUST 50 // software
#define DEFAULT_LCD_BRIGHTNESS 0 // hardware, does nothing to HDMI, 0 FULL brightnes, 1023 OFF
#define GLOBAL_OVERLAY_ALPHA 50

#define SIMPLE_UNDRAW 0 // if "non" simple, then "stable" vectors are not drawn again - but stay over frames

#define HUGE_DATA_LOCATION EXT_RAM_BSS_ATTR // /*DRAM_ATTR*/EXT_RAM_BSS_ATTR  

#define FLASH_ROM_ATTR __attribute__((section(".rodata")))

// audio quality!
#define AY_FREQUENCY 44100 // samples per second
#define AY_CHANNEL 2 // 1 = mono, 2 = stereo
#define AY_BITS 16 // not working atm with other then 16, ES8311 only support 16 bit or higher... dont set it any other then 16!

#define VIDEO_OUT_HDMI 1
#define VIDEO_OUT_LVDS 2

#define isHDMI() ((mode&VIDEO_OUT_HDMI)==VIDEO_OUT_HDMI)
#define isLVDS() ((mode&VIDEO_OUT_LVDS)==VIDEO_OUT_LVDS)

#define VIDEO_OUT_SELECTED VIDEO_OUT_LVDS   // this is the startup configuration

#define MAX_LINE_BUFFER   1000
#define NUM_FB            2        // Hardware-Framebuffer
#define NUM_FRAME_SLOTS   3        // Logische Emu-Frames (Linien)

#define INI_FILE_PATH   "/sdcard/ESP.INI"

#define MAX_OVERLAY_NAME 128

// the overlay HDMI LANDSCAPE (default)
#define HDMI_OVERLAY_WIDTH 564		
#define HDMI_OVERLAY_HEIGHT 720		

// the vecterx output inside an overlay
#define HDMI_IN_OVERLAY_VECX_WIDTH 500		
#define HDMI_IN_OVERLAY_VECX_HEIGHT 700	

#define HDMI_VECX_WIDTH 500		
#define HDMI_VECX_HEIGHT 720

// the overlay HDMI PORTRAIT
#define HDMI_PORTRAIT_OVERLAY_WIDTH 720		
#define HDMI_PORTRAIT_OVERLAY_HEIGHT 960

// the vecterx output inside an overlay
#define HDMI_PORTRAIT_IN_OVERLAY_VECX_WIDTH 700		
#define HDMI_PORTRAIT_IN_OVERLAY_VECX_HEIGHT 930

#define HDMI_PORTRAIT_VECX_WIDTH 720
#define HDMI_PORTRAIT_VECX_HEIGHT 1040 //960



// the overlay LCD PORTRAIT default
#define LCD_OVERLAY_WIDTH 800		
#define LCD_OVERLAY_HEIGHT 480		

// the vecterx output inside an overlay
#define LCD_IN_OVERLAY_VECX_WIDTH 400 // 410		
#define LCD_IN_OVERLAY_VECX_HEIGHT 700		

// if no overlay - vectrex fills the screen
#define LCD_VECX_WIDTH 480 // 410		
#define LCD_VECX_HEIGHT 800		

// the overlay LCD LANDSCAPE
#define LCD_LANDSCAPE_OVERLAY_WIDTH 480
#define LCD_LANDSCAPE_OVERLAY_HEIGHT 360		

// the vecterx output inside an overlay
#define LCD_LANDSCAPE_IN_OVERLAY_VECX_WIDTH 380 // 410		
#define LCD_LANDSCAPE_IN_OVERLAY_VECX_HEIGHT 340

// if no overlay - vectrex fills the screen
#define LCD_LANDSCAPE_VECX_WIDTH 480 // 410		
#define LCD_LANDSCAPE_VECX_HEIGHT 360		

#define ORIENTATION_0 0
#define ORIENTATION_90 1
#define ORIENTATION_180 2
#define ORIENTATION_270 3



// #define FLASH_SUPPORT - not tested
// #define MOVIE_SUPPORT - not tested


// globals
extern int mode;          // defined in main.c
extern int overlayEnabled;
extern int LCD_H_RES;
extern int LCD_V_RES;

extern int g_color_mode; // 0 = not color, 1 = color
extern int g_line_width;
extern int g_line_glow;
extern int g_Orientation;
extern int brightnessAdjust;
extern long unsigned int g_beam_r2_q16;

extern int g_line_Rb;
extern int g_beam_r;
extern long unsigned int g_gs;

extern int brightnessLCD;
extern const uint8_t gauss_lut[];
extern uint8_t s_overlay_palette[128][3];
extern int g_fpsToReach;

typedef struct {
    volatile int8_t j0_x;
    volatile int8_t j0_y;
    volatile int8_t j1_x;
    volatile int8_t j1_y;
    volatile int buttonState;
} input_state_t;
extern volatile input_state_t g_inputState; 


//void emu_begin_frame(void);
void mini_end_frame(void);
void mini_draw_line(int x0, int y0, int x1, int y1, uint8_t brightness); // brightness or color - depending on mode, 0 is always undraw
void mini_draw_line_color(int x0, int y0, int x1, int y1, int color, uint8_t brightness);

//void vecx_emu (long cycles);
int getDisplayWidth();
int getDisplayHeight();
int getScreenWidth();
int getScreenHeight();
void setAppFPS(int fps);


//DRAM_ATTR int16_t audioBuffer[AUDIO_FRAME_SAMPLES];
/* Callback-Typ: mono, 16 Bit, length = Anzahl Samples */
typedef void (*audio_sample_callback_t)(void *userdata, int16_t *stream, int length);
void audio_set_callback(audio_sample_callback_t cb, void *userdata);

bool read_ini_rom_name(char *out_name, int out_size);
long load_rom_file(const char *rom_name, uint8_t *buffer, long max_size);

bool isKeyDown(uint8_t hid_code);
bool isAsciiDown(char c);
esp_err_t loadOverlayRGB(char *name);


// different input devices can be plugged in at the same time
// following is the order in which they are read
// if a value is read the "next" device is not queried anymore!
// Joystick_9Pin > USBJoystick > USBKeyboard

extern int mini9PinButton; // zero active
extern int miniVideoButton;
extern int miniResetButton;
extern int miniHPD;
extern int miniSD;

// Analog Joystick in 9 pin plug
// you probably do not need them
// values are read in FrameEnd 
// and are available in g_inputState
bool is9PinJoystickAvailable(void);
void get9PinAnalog(void);
int get9PinAnalogX(void); // values from -128 - +127
int get9PinAnalogY(void);
int getSwitches(void); // retunrs 9pin buttons if available, but sets all 8 GPIO

extern int miniUSBButton; // zero active
bool isUSBJoystickAvailable(void);
int getUSBAnalogX(void); // values from -128 - +127
int getUSBAnalogY(void);






/* ── Palette index defines ─────────────────────────────────────────────── */

/* Neutrals */
#define YUV_PALETTE_BLACK           0
#define YUV_PALETTE_DARKGRAY        1
#define YUV_PALETTE_GRAY            2
#define YUV_PALETTE_MIDGRAY         3
#define YUV_PALETTE_LIGHTGRAY       4
#define YUV_PALETTE_SILVER          5
#define YUV_PALETTE_OFFWHITE        6
#define YUV_PALETTE_WHITE           7

/* Reds */
#define YUV_PALETTE_DARKMAROON      8
#define YUV_PALETTE_MAROON          9
#define YUV_PALETTE_DARKRED        10
#define YUV_PALETTE_RED            11
#define YUV_PALETTE_BRIGHTRED      12
#define YUV_PALETTE_CORAL          13
#define YUV_PALETTE_SALMON         14
#define YUV_PALETTE_LIGHTCORAL     15

/* Oranges / Yellows */
#define YUV_PALETTE_DARKORANGE     16
#define YUV_PALETTE_ORANGE         17
#define YUV_PALETTE_BRIGHTORANGE   18
#define YUV_PALETTE_GOLD           19
#define YUV_PALETTE_YELLOW         20
#define YUV_PALETTE_LIGHTYELLOW    21
#define YUV_PALETTE_AMBER          22
#define YUV_PALETTE_KHAKI          23

/* Greens */
#define YUV_PALETTE_DARKFOREST     24
#define YUV_PALETTE_FOREST         25
#define YUV_PALETTE_DARKGREEN      26
#define YUV_PALETTE_GREEN          27
#define YUV_PALETTE_BRIGHTGREEN    28
#define YUV_PALETTE_LIGHTGREEN     29
#define YUV_PALETTE_PALEGREEN      30
#define YUV_PALETTE_YELLOWGREEN    31

/* Cyans / Teals */
#define YUV_PALETTE_DARKTEAL       32
#define YUV_PALETTE_TEAL           33
#define YUV_PALETTE_DARKCYAN       34
#define YUV_PALETTE_CYAN           35
#define YUV_PALETTE_LIGHTCYAN      36
#define YUV_PALETTE_AQUAMARINE     37
#define YUV_PALETTE_TURQUOISE      38
#define YUV_PALETTE_MEDTURQUOISE   39

/* Blues */
#define YUV_PALETTE_DARKNAVY       40
#define YUV_PALETTE_NAVY           41
#define YUV_PALETTE_DARKBLUE       42
#define YUV_PALETTE_BLUE           43
#define YUV_PALETTE_ROYALBLUE      44
#define YUV_PALETTE_CORNFLOWER     45
#define YUV_PALETTE_SKYBLUE        46
#define YUV_PALETTE_LIGHTBLUE      47

/* Purples / Violets */
#define YUV_PALETTE_DARKINDIGO     48
#define YUV_PALETTE_INDIGO         49
#define YUV_PALETTE_DARKPURPLE     50
#define YUV_PALETTE_PURPLE         51
#define YUV_PALETTE_VIOLET         52
#define YUV_PALETTE_BLUEVIOLET     53
#define YUV_PALETTE_MEDIUMPURPLE   54
#define YUV_PALETTE_LAVENDER       55

/* Pinks / Magentas */
#define YUV_PALETTE_DARKMAGENTA    56
#define YUV_PALETTE_MAGENTA        57
#define YUV_PALETTE_FUCHSIA        58
#define YUV_PALETTE_DEEPPINK       59
#define YUV_PALETTE_HOTPINK        60
#define YUV_PALETTE_PINK           61
#define YUV_PALETTE_LIGHTPINK      62
#define YUV_PALETTE_ROSE           63

/* Browns / Earth */
#define YUV_PALETTE_DARKBROWN      64
#define YUV_PALETTE_BROWN          65
#define YUV_PALETTE_SADDLEBROWN    66
#define YUV_PALETTE_CHOCOLATE      67
#define YUV_PALETTE_PERU           68
#define YUV_PALETTE_TAN            69
#define YUV_PALETTE_WHEAT          70
#define YUV_PALETTE_SANDYBROWN     71

/* Olive / Sea greens */
#define YUV_PALETTE_OLIVEDARK      72
#define YUV_PALETTE_OLIVE          73
#define YUV_PALETTE_DARKOLIVE      74
#define YUV_PALETTE_OLIVEDRAB      75
#define YUV_PALETTE_SEAGREEN       76
#define YUV_PALETTE_MEDSEAGREEN    77
#define YUV_PALETTE_SPRINGGREEN    78
#define YUV_PALETTE_MINTGREEN      79

/* Blue-greens / Slate */
#define YUV_PALETTE_CADETBLUE      80
#define YUV_PALETTE_STEELBLUE      81
#define YUV_PALETTE_DODGERBLUE     82
#define YUV_PALETTE_DEEPSKYBLUE    83
#define YUV_PALETTE_POWDERBLUE     84
#define YUV_PALETTE_SLATEBLUE      85
#define YUV_PALETTE_MIDNIGHTBLUE   86
#define YUV_PALETTE_DARKSLATEBLUE  87

/* Warm reds */
#define YUV_PALETTE_CRIMSON        88
#define YUV_PALETTE_SCARLET        89
#define YUV_PALETTE_TOMATO         90
#define YUV_PALETTE_FIREBRICK      91
#define YUV_PALETTE_INDIANRED      92
#define YUV_PALETTE_ROSEWOOD       93
#define YUV_PALETTE_RUBY           94
#define YUV_PALETTE_BURGUNDY       95

/* Pastels */
#define YUV_PALETTE_PASTELRED      96
#define YUV_PALETTE_PASTELORANGE   97
#define YUV_PALETTE_PASTELYELLOW   98
#define YUV_PALETTE_PASTELGREEN    99
#define YUV_PALETTE_PASTELCYAN    100
#define YUV_PALETTE_PASTELBLUE    101
#define YUV_PALETTE_PASTELPURPLE  102
#define YUV_PALETTE_PASTELPINK    103

/* Neons */
#define YUV_PALETTE_NEONRED       104
#define YUV_PALETTE_NEONORANGE    105
#define YUV_PALETTE_NEONYELLOW    106
#define YUV_PALETTE_NEONGREEN     107
#define YUV_PALETTE_NEONCYAN      108
#define YUV_PALETTE_NEONBLUE      109
#define YUV_PALETTE_NEONPURPLE    110
#define YUV_PALETTE_NEONPINK      111

/* Electric / Saturated */
#define YUV_PALETTE_ELECTRICBLUE  112
#define YUV_PALETTE_ELECTRICGREEN 113
#define YUV_PALETTE_ELECTRICPURP  114
#define YUV_PALETTE_ELECTRICCYAN  115
#define YUV_PALETTE_ELECTRICYELL  116
#define YUV_PALETTE_ELECTRICORANG 117
#define YUV_PALETTE_ELECTRICRED   118
#define YUV_PALETTE_ELECTRICPINK  119

/* Misc warm */
#define YUV_PALETTE_OCHRE         120
#define YUV_PALETTE_TERRACOTTA    121
#define YUV_PALETTE_RUST          122
#define YUV_PALETTE_COPPER        123
#define YUV_PALETTE_BRONZE        124
#define YUV_PALETTE_BRASS         125
#define YUV_PALETTE_CHARTREUSE    126
#define YUV_PALETTE_LIMEGREEN     127
