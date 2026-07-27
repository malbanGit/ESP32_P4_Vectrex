#include "../defines.h"


#ifndef __GNUC__
#define inline
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "memory.h"
#include "game.h"
#include "display.h"
#include "sim6502.h"
#include "esp_timer.h"
#include "macro6502.h"

#include "pokey.h"


// registers of the 6502 - set before executing
extern int PC;
extern int A;
extern int X;
extern int Y;
extern long totcycles;
extern int CC;


void yuv_palette_init(void);
void setAppFPS(int fps);
void avg_reset_really();
void audio_set_callback(audio_sample_callback_t cb, void *userdata);
void callbackPokey(void *userdata, int16_t *stream, int length);

HUGE_DATA_LOCATION char gameMemory[4*65536];
DRAM_ATTR int tobekilled = 0;
int a_fps = 60;

void init_graphics (); // framework.c

// listeners will be called at the end of a frame!
// The listener — receives events from the ESP32 environment layer.
// Only acts on the types it cares about; ignores everything else.
static void esp_event_listener(const ESPEvent *event)
{
    switch (event->type)
    {
		case ESP_EVENT_INIT:
        init_graphics(); 
			break;
		case ESP_EVENT_SIZE_CHANGED:
        init_graphics();
			break;
		case ESP_EVENT_KILL:
			  tobekilled = 1;
			break;
    default:
        break;   // not our event — ignore
    }
}

void initArcadia()
{
  mem = (elem *) gameMemory;
  game = pick_game (ProgName);

  init_graphics();
  setAppFPS(a_fps);
  setup_game();
  

  audio_set_callback(callbackPokey, NULL);
  esp_add_event_listener(esp_event_listener);

  // init 6502 emulation
  // pc and IRQ are game specific
  totcycles = 0;
  A = 0;
  X = 0;
  Y = 0;
  byte_to_flags (0);
  CC=0; SET_I; 
  avg_reset_really();
  tobekilled = 0;
}
void deinitArcadia()
{
  audio_set_callback(NULL, NULL);
  esp_remove_event_listener(esp_event_listener);
}


int tempest(void)
{
  ProgName = "tempest"; // defined in display.h
  ALG_XMIN=-16000;
  ALG_XMAX=16000;
  ALG_YMIN=-16000;
  ALG_YMAX=16000;
  
  ALG_MAX_X=30000; // wid
  ALG_MAX_Y=36000;

  POS_ADDER_X = 15000;
  POS_ADDER_Y = 14000;
  FLIP_Y=1;
  SCREEN_TYPE = GAME_PORTRAIT;
  a_fps = 600; // sets g_fpsToReach

  initArcadia();
  g_color_mode = 1;
  yuv_palette_init();

  PC = (memrd(0xfffd,0,0) << 8) | memrd(0xfffc,0,0);
  irq_cycle = 4173; // tempest 

	while (1)
	{
      uint64_t start = esp_timer_get_time();
      sim_6502 (25200); // 1512000 Mhz / 60 FPS

      // slow down if we are too fast
      // emulating 1 frame should take 1/50 of a second...
      // if MAX_EMU_FPS is 60 and 60 is reached, we run with 100% original speed
      uint64_t now = esp_timer_get_time();
      if (now - start <= g_fpsToReach) 
      {
          int delay = g_fpsToReach - (now - start);
          esp_rom_delay_us(delay);  
      }
      
      if (tobekilled) break;
	}
  deinitArcadia();
  return 0;
}

int battlezone(void)
{
  ProgName = "battlezone"; // defined in display.h
  mem = (elem *) gameMemory;
  game = pick_game (ProgName);

  ALG_XMIN=0;
  ALG_XMAX=1000;
  ALG_YMIN=0;
  ALG_YMAX=750;
  ALG_MAX_X=1000; // wid
  ALG_MAX_Y=750;

  POS_ADDER_X = 0;
  POS_ADDER_Y = 0;
  FLIP_Y=0;
  SCREEN_TYPE = GAME_LANDSCAPE;


  init_graphics ();
  setAppFPS(41);
  printf("Game picked: %i\n", game);
  setup_game();
  g_color_mode = 0;
 
  save_PC = (memrd(0xfffd,0,0) << 8) | memrd(0xfffc,0,0);
  save_A = 0;
  save_X = 0;
  save_Y = 0;
  save_flags = 0;
  save_totcycles = 0;
  irq_cycle = 8192; // battlezone

  avg_reset_really();
  while (1)
  {
  sim_6502 (25200); // 1512000 Mhz / 60 FPS
  }
  return 0;
}
int blackwidow(void)
{
  ProgName = "blackwidow"; // defined in display.h
  
  mem = (elem *) gameMemory;
  game = pick_game (ProgName);

  ALG_XMIN=0;
  ALG_XMAX=1100;
  ALG_YMIN=0;
  ALG_YMAX=850;
  ALG_MAX_X=1100; // wid
  ALG_MAX_Y=850;

  POS_ADDER_X = 0;
  POS_ADDER_Y = 0;
  FLIP_Y=0;
  SCREEN_TYPE = GAME_LANDSCAPE;

  init_graphics ();
  setAppFPS(60);
  setup_game();
  g_color_mode = 1;
  yuv_palette_init();
  // from pokey.c
  //  void enablePokeyOutput(int e);
  //  enablePokeyOutput(0); // false
  
  save_PC = (memrd(0xfffd,0,0) << 8) | memrd(0xfffc,0,0);
  save_A = 0;
  save_X = 0;
  save_Y = 0;
  save_flags = 0;
  save_totcycles = 0;
  irq_cycle = 8192; // blackwidow

  avg_reset_really();
  while (1)
  {
  sim_6502 (25200); // 1512000 Mhz / 60 FPS
  }
  return 0;
}
int deluxe(void)
{
  ProgName = "deluxe"; // defined in display.h
  mem = (elem *) gameMemory;
  game = pick_game (ProgName);

  ALG_XMIN=0;
  ALG_XMAX=1000;
  ALG_YMIN=0;
  ALG_YMAX=750;
  ALG_MAX_X=1000; // wid
  ALG_MAX_Y=750;

  POS_ADDER_X = 0;
  POS_ADDER_Y = 0;
  FLIP_Y=0;
  SCREEN_TYPE = GAME_LANDSCAPE;


  init_graphics ();
  setAppFPS(41);
  printf("Game picked: %i\n", game);
  setup_game();
  g_color_mode = 0;
 
  save_PC = (memrd(0xfffd,0,0) << 8) | memrd(0xfffc,0,0);
  save_A = 0;
  save_X = 0;
  save_Y = 0;
  save_flags = 0;
  save_totcycles = 0;
  irq_cycle = 8192; // battlezone

  avg_reset_really();
  while (1)
  {
  sim_6502 (25200); // 1512000 Mhz / 60 FPS
  }
  return 0;
}
int gravitar(void)
{
  ProgName = "gravitar"; // defined in display.h
  
  mem = (elem *) gameMemory;
  game = pick_game (ProgName);

  ALG_XMIN=0;
  ALG_XMAX=1100;
  ALG_YMIN=0;
  ALG_YMAX=850;
  ALG_MAX_X=1100; // wid
  ALG_MAX_Y=850;

  POS_ADDER_X = 0;
  POS_ADDER_Y = 0;
  FLIP_Y=0;
  SCREEN_TYPE = GAME_LANDSCAPE;

  init_graphics ();
  setAppFPS(60);
  setup_game();
  g_color_mode = 1;
  yuv_palette_init();
  // from pokey.c
  //  void enablePokeyOutput(int e);
  //  enablePokeyOutput(0); // false
  
  save_PC = (memrd(0xfffd,0,0) << 8) | memrd(0xfffc,0,0);
  save_A = 0;
  save_X = 0;
  save_Y = 0;
  save_flags = 0;
  save_totcycles = 0;
  irq_cycle = 8192; // blackwidow

  avg_reset_really();
  while (1)
  {
  sim_6502 (25200); // 1512000 Mhz / 60 FPS
  }
  return 0;
}
int lunar(void)
{
  ProgName = "lunar"; // defined in display.h
  mem = (elem *) gameMemory;
  game = pick_game (ProgName);

  ALG_XMIN=0;
  ALG_XMAX=1000;
  ALG_YMIN=0;
  ALG_YMAX=750;
  ALG_MAX_X=1000; // wid
  ALG_MAX_Y=750;

  POS_ADDER_X = 0;
  POS_ADDER_Y = 0;
  FLIP_Y=0;
  SCREEN_TYPE = GAME_LANDSCAPE;


  init_graphics ();
  setAppFPS(41);
  printf("Game picked: %i\n", game);
  setup_game();
  g_color_mode = 0;
 
  save_PC = (memrd(0xfffd,0,0) << 8) | memrd(0xfffc,0,0);
  save_A = 0;
  save_X = 0;
  save_Y = 0;
  save_flags = 0;
  save_totcycles = 0;
  irq_cycle = 6136; // battlezone

  avg_reset_really();
  while (1)
  {
  sim_6502 (25200); // 1512000 Mhz / 60 FPS
  }
  return 0;
}
int redbaron(void)
{
  ProgName = "redbaron"; // defined in display.h
  mem = (elem *) gameMemory;
  game = pick_game (ProgName);

  ALG_XMIN=0;
  ALG_XMAX=1000;
  ALG_YMIN=0;
  ALG_YMAX=750;
  ALG_MAX_X=1000; // wid
  ALG_MAX_Y=750;

  POS_ADDER_X = 0;
  POS_ADDER_Y = 0;
  FLIP_Y=0;
  SCREEN_TYPE = GAME_LANDSCAPE;


  init_graphics ();
  setAppFPS(62);
  printf("Game picked: %i\n", game);
  setup_game();
  g_color_mode = 0;
 
  save_PC = (memrd(0xfffd,0,0) << 8) | memrd(0xfffc,0,0);
  save_A = 0;
  save_X = 0;
  save_Y = 0;
  save_flags = 0;
  save_totcycles = 0;
  irq_cycle = 8192; // battlezone

  avg_reset_really();
  while (1)
  {
  sim_6502 (25200); // 1512000 Mhz / 60 FPS
  }
  return 0;
}
int asteroids(void)
{
  ProgName = "asteroids"; // defined in display.h
  mem = (elem *) gameMemory;
  game = pick_game (ProgName);

  ALG_XMIN=0;
  ALG_XMAX=1000;
  ALG_YMIN=0;
  ALG_YMAX=750;
  ALG_MAX_X=1000; // wid
  ALG_MAX_Y=750;

  POS_ADDER_X = 0;
  POS_ADDER_Y = 0;
  FLIP_Y=0;
  SCREEN_TYPE = GAME_LANDSCAPE;


  init_graphics ();
  setAppFPS(62);
  printf("Game picked: %i\n", game);
  setup_game();
  g_color_mode = 0;
 
  save_PC = (memrd(0xfffd,0,0) << 8) | memrd(0xfffc,0,0);
  save_A = 0;
  save_X = 0;
  save_Y = 0;
  save_flags = 0;
  save_totcycles = 0;
  irq_cycle = 8192; // 

  avg_reset_really();
  while (1)
  {
  sim_6502 (25200); // 1512000 Mhz / 60 FPS
  }
  return 0;
}
int spaceDuel(void)
{
  ProgName = "spaceduel"; // defined in display.h
  
  mem = (elem *) gameMemory;
  game = pick_game (ProgName);

  ALG_XMIN=0;
  ALG_XMAX=1100;
  ALG_YMIN=0;
  ALG_YMAX=850;
  ALG_MAX_X=1100; // wid
  ALG_MAX_Y=850;

  POS_ADDER_X = 0;
  POS_ADDER_Y = 0;
  FLIP_Y=0;
  SCREEN_TYPE = GAME_LANDSCAPE;

  init_graphics ();
  setAppFPS(60);
  setup_game();
  g_color_mode = 1;
  yuv_palette_init();
  // from pokey.c
  //  void enablePokeyOutput(int e);
  //  enablePokeyOutput(0); // false
  
  save_PC = (memrd(0xfffd,0,0) << 8) | memrd(0xfffc,0,0);
  save_A = 0;
  save_X = 0;
  save_Y = 0;
  save_flags = 0;
  save_totcycles = 0;
  irq_cycle = 8192; // blackwidow

  avg_reset_really();
  while (1)
  {
  sim_6502 (25200); // 1512000 Mhz / 60 FPS
  }
  return 0;
}

