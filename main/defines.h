#pragma once

//#define NO_AUDIO 1 just be quiet!

#define VECX_DEBUG 1
#define MAX_EMU_FPS 50
#define ENABLE_OVERLAYS 1
#define LINE_WIDTH 1
#define LINE_GLOW_WIDTH 2
#define BRIGHTNESS_ADJUST 150
#define GLOBAL_OVERLAY_ALPHA 50

#define SIMPLE_UNDRAW 0 // if "non" simple, then "stable" vectors are not drawn again - but stay over frames

#define HUGE_DATA_LOCATION EXT_RAM_BSS_ATTR // /*DRAM_ATTR*/EXT_RAM_BSS_ATTR  

// audio quality!
#define AY_FREQUENCY 44100 // samples per second
#define AY_CHANNEL 2 // 1 = mono, 2 = stereo
#define AY_BITS 16 // not working atm with other then 16, ES8311 only support 16 bit or higher... dont set it any other then 16!

//typedef enum { VIDEO_OUT_HDMI, VIDEO_OUT_LVDS } video_out_t;

#define VIDEO_OUT_HDMI 0
#define VIDEO_OUT_LVDS 1

#define VIDEO_OUT_SELECTED VIDEO_OUT_LVDS

#if VIDEO_OUT_SELECTED == VIDEO_OUT_HDMI
#define AUDIO_OUT_HDMI  1 /* 0 = onboard speaker (I2S0 + ES8311), 1 = HDMI audio (I2S1 + LT8912B) */
#else
#define AUDIO_OUT_HDMI  0 /* 0 = onboard speaker (I2S0 + ES8311), 1 = HDMI audio (I2S1 + LT8912B) */
#endif



#define MAX_LINE_BUFFER   1000
#define NUM_FB            2        // Hardware-Framebuffer
#define NUM_FRAME_SLOTS   3        // Logische Emu-Frames (Linien)

#define INI_FILE_PATH   "/sdcard/ESP.INI"

// vectrex emu
#define MAX_ROM_NAME    128
#define MAX_CART_SIZE   32768*2*4 // 32768*2*4 for vectorblade only when Data is in PSRAM, otherwise too large!
#define DEFAULT_AUTO_SYNC 1 // !! autosync only works reliably when WaitRecal is used. Spike speach out - does not use WR -> frames are missed!!!

// #define FLASH_SUPPORT - not tested
// #define MOVIE_SUPPORT - not tested


// following is defined in board.h
// #define VIDEO_FB_YUV422          0 // YUV only supported when overlays are disabled


