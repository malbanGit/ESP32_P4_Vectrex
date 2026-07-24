#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_attr.h"

//#define NO_AUDIO 1 just be quiet!

#define VECX_DEBUG 1
#define MAX_EMU_FPS 50
#define ENABLE_OVERLAYS 0

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

#define VIDEO_OUT_HDMI 0
#define VIDEO_OUT_LVDS 1
#define VIDEO_OUT_SELECTED VIDEO_OUT_LVDS   // this is the startup configuration


#define MAX_LINE_BUFFER   1000
#define NUM_FB            2        // Hardware-Framebuffer
#define NUM_FRAME_SLOTS   3        // Logische Emu-Frames (Linien)

#define INI_FILE_PATH   "/sdcard/ESP.INI"

// vectrex emu
#define MAX_ROM_NAME    128
#define MAX_CART_SIZE   32768*2*4 // 32768*2*4 for vectorblade only when Data is in PSRAM, otherwise too large!
#define DEFAULT_AUTO_SYNC 1 // !! autosync only works reliably when WaitRecal is used. Spike speach out - does not use WR -> frames are missed!!!

// the overlay
#define HDMI_OVERLAY_WIDTH 564		
#define HDMI_OVERLAY_HEIGHT 720		

// the vecterx output inside an overlay
#define HDMI_IN_OVERLAY_VECX_WIDTH 500		
#define HDMI_IN_OVERLAY_VECX_HEIGHT 700	

#define HDMI_VECX_WIDTH 500		
#define HDMI_VECX_HEIGHT 720

// the overlay
#define LCD_OVERLAY_WIDTH 800		
#define LCD_OVERLAY_HEIGHT 480		

// the vecterx output inside an overlay
#define LCD_IN_OVERLAY_VECX_WIDTH 400 // 410		
#define LCD_IN_OVERLAY_VECX_HEIGHT 700		

// if now overlay - vectrex fills the screen
#define LCD_VECX_WIDTH 480 // 410		
#define LCD_VECX_HEIGHT 800		





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
    int8_t j0_x;
    int8_t j0_y;
    int8_t j1_x;
    int8_t j1_y;
    int buttonState;
} input_state_t;
extern input_state_t g_inputState; 


//void emu_begin_frame(void);
void mini_end_frame(void);
void mini_draw_line(int x0, int y0, int x1, int y1, uint8_t brightness); // brightness or color - depending on mode, 0 is always undraw
void mini_draw_line_color(int x0, int y0, int x1, int y1, int color, uint8_t brightness);
//void vecx_emu (long cycles);
int getDisplayWidth();
int getDisplayHeight();
int getScreenWidth();
int getScreenHeight();














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
