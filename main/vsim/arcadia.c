#include "../defines.h"
#include "pokey.h"

#include "esp_timer.h"
#include "memory.h"
#include "macro6502.h"


HUGE_DATA_LOCATION char gameMemory[4*65536];
void yuv_palette_init(void);
void setAppFPS(int fps);
DRAM_ATTR int tobekilled = 0;


void init_graphics (); // framework.c
extern int vecsimSigDone;

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
			vecsimSigDone = 1;
			break;
    default:
        break;   // not our event — ignore
    }
}


void avg_reset_really();

/*
https://github.com/historicalsource/tempest
https://arcarc.xmission.com/Web%20Archives/ionpool.net%20(Dec-31-2020)/arcade/tempest_code_project/tempest_code_project.html
https://github.com/Threetwosevensixseven/ayfxedit-improved
;
; POKEY DEFINITIONS
;
AUDF1	=POKEY
AUDC1	=POKEY+1
AUDCTL	=POKEY+8
ALLPOT	=POKEY+8
RANDOM	=POKEY+0A
POTGO	=POKEY+0B
SKCTL	=POKEY+0F
AUDF2	=POKEY2
AUDC2	=POKEY2+1
AUD2CTL	=POKEY2+8
ALLPO2	=POKEY2+8
RANDO2	=POKEY2+0A
POTGO2	=POKEY2+0B
SKCTL2	=POKEY2+0F

CPEXPL:	LDA I,SIDDI		;PLAYER DIES
	JMP SNDON
 -> 
CCB0 A9:5F      LDA:imm    #5F
CCB2 4C:C3 CC   JMP:abs    $CCC3



;EXPLOSION SOUND
;
T51F:	.BYTE 0C0,8,4,10
	.BYTE 0,0
T51A:	.BYTE 0A6,20,0F8,4
	.BYTE 0,0
T52F:	.BYTE 40,8,4,10
	.BYTE 0,0
T52A:	.BYTE 0A6,20,0FE,4
	.BYTE 0,0

CBD1 C0:08      CPY:imm    #08
CBD3 04:        Illegal Opcode
CBD4 10:00      BPL:rel    Branch->$CBD6
CBD6 00:        BRK:imp    BREAK
CBD7 A6:20      LDX:zp     Zp RAM 0020
CBD9 F8:        SED:imp    Set Decimal
CBDA 04:        Illegal Opcode
CBDB 00:        BRK:imp    BREAK
CBDC 00:        BRK:imp    BREAK
CBDD 40:        RTI:imp    Ret from Int
CBDE 08:        PHP:imp    Push P
CBDF 04:        Illegal Opcode
CBE0 10:00      BPL:rel    Branch->$CBE2
CBE2 00:        BRK:imp    BREAK
CBE3 A6:20      LDX:zp     Zp RAM 0020
CBE5 FE:04 00   INC:abs,x  $0004,X
CBE8 00:        BRK:imp    BREAK


Table of Calculated Addresses: Chatty
Label   Address

CPEXPL  $CCB0; player dies
SBOING  $CCB5; Cursor moves
SAUSON  $CCB9; Score Sound
ESLSON  $CCBD; Enemy Shot
CCEXPL  $CCC1; Explosion 
CIEXPL  $CCC1
EXSNON  $CCC1

; sound routines
- SNDON   $CCC3 - sounds only when not in attract mode!
- FSNDON  $CCC5
- NWSNON  $CCE8


SLAUNC  $CCE9; launch sound, player fire
SOUTS2  $CCED; thrust on tube
SOUTS3  $CCF1; thrust in space
SELICO  $CCF5; enemy line destruction sound
SSLAMS  $CCF9; slam sound
S3SWAR  $CCFD; 3 second warning
PULSTR  $CD01; pulsation ON
PULSTO  $CD05; Pulsation OFF

Colors: from file: ALCOMN.MAC
;
;COLORS
;
BLUE=6
BLULET  =7
GREEN=5
RED=3
YELLOW=1
WHITE=0
PURPLE=2
TURQOI=4
WELCOL=BLUE         ;WELL
CURCOL=YELLOW           ;CURSOR
ICHCOL=WHITE            ;ENEMY SHOTS
PCHCOL=YELLOW           ;PLAYER SHOTS
INVCOL=RED          ;INVADERS
LETCOL=GREEN            ;LETTERS
DEPCOL=WELCOL
EXPCOL=WHITE            ;EXPLOSION
FLICOL=RED          ;FLIPPERS
TANCOL=PURPLE           ;TANKER
TRACOL=GREEN            ;TRALERS
ZAPCOL=WHITE            ;SUPER ZAP
FRED    =0C
FBLUE   =0B
FGREEN  =07
HRED    =0D
ZWHITE  =FRED&FBLUE&FGREEN
ZYELLO  =FRED&FGREEN
ZPURPL  =FRED&FBLUE
ZRED    =FRED
ZTURQOI =FGREEN&FBLUE
ZGREEN  =FGREEN
ZBLUE   =FBLUE
ZBLACK  =0F
PSHCTR  =8      ;PLAYER SOT CENTER
PDIWHI  =9          ;PLAYER DEATH EXPLOSION COLORS
PDIYEL  =10.
PDIRED  =11.
NYMCOL  =12.            ;NYMPHE
FLASH   =15.            ;CHANGES EVERY 4 MO.
    .PAGE
    .SBTTL VARIABLES-CONTROL    
    

 * main.c: Atari Vector game simulator
 *
 * Copyright 1991-1993, 1996, 2003 Hedley Rainnie, Doug Neubauer, and Eric Smith
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * $Id: main.c,v 1.1 2018/07/31 01:19:45 pi Exp $
 */

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


// RAM addresses for Tempest

// 00 During attract, C0 during gameplay, 80 during high score entry.
/*
MATRACT =80     ;D7: 0=ATTRACT  1=GAME
MGTMOD  =40     ;D6: 0=NO TIME;START ALLOWED
                ;  : 1GAME TIMER RUNNING
                ;  : PRESS START NOT ALLOWED
*/                
//STATUS FLAGS
#define QSTATUS 0x0005 

/*
;
;QSTATE CODES (ROUTAD INDICES)
;
CNEWGA  =0 -> new game
CNEWLI  =2
CPLAY   =4 -> well near enough and player "playing"
CENDLI  =6
CENDGA  =8
CPAUSE  =0A
CNEWAV  =0C -> next state is CNEWV2
CENDWAV =0E
CHISCHK =10
CDLADR  =14
CGETINI =12
CNOTFOU =CDLADR
CREQRAT =CDLADR+2
CNEWV2  =CREQRAT+2 ; new wave after first init

CLOGO   =CNEWV2+2       ;LOGO INIT
CINIRAT =CLOGO+2
CNWLF2  =CINIRAT+2
CDROP   =CNWLF2+2       ;DROP MODE
CSYSTM  =CDROP+2
CBOOM   =CSYSTM+2       ;BOOM STATE


;
;STATE ROUTINE ADDRESS
;
ROUTAD: .WORD NEWGAM-1      ;NEW GAME
    .WORD NEWLIF-1      ;NEW LIFE (AFTER LOSING A BASE)
    .WORD PLAY-1        ;PLAY
    .WORD ENDLIF-1      ;LIFE LOST
    .WORD ENDGAM-1      ;END OF GAME
    .WORD PAUSE-1       ;PAUSE
    .WORD 0         ;NEW WAVE (AFTER SHOOTING ALL INVADERS)
    .WORD ENDWAV-1      ;END OF WAVE
    .WORD HISCHK-1      ;CHECK FOR HI SCORES
    .WORD GETINI-1      ;GET HI SCORE INITIALS
    .WORD DLADR-1       ;DISPLAY HI SCORE TABLE
    .WORD PRORAT-1      ;REQUEST PLAYER RATE
    .WORD NEWAV2-1      ;NEW WAVE PART 2
    .WORD LOGINI-1      ;LOGO INIT
    .WORD INIRAT-1      ;MONSTER DELAY/DISPLAY
    .WORD NEWLF2-1      ;NEW LIFE PART 2
    .WORD PLDROP-1      ;DROP MODE
    .WORD SYSTEM-1      ;END WAVE CLEAN UP AFTER BONUS
    .WORD PRBOOM-1      ;BOOM
ROUTEN:

*/
// ;CONTAINS CODE FOR STATE ROUTINE (INDEX INTO ROUTAD)
#define QSTATE 0x0000 


/*
;
;DISPLAY STATE CODES
CDPLAY  =0          ;PLAY
CDSYST  =2          ;SYSTEM CONFIGURATION
CDREQRA =8          ;REQUEST RANK
CDPLPL  =0E         ;PLAY PLAYER WARNING

CDGOVR  =0C         ;GAME OVER PLAYER MSG
CDHITB  =0A         ;HI SCORE TABLE
CDGETI  =6          ;GET INITIALS
CDBOOM  =4          ;BOOM DISPLAY
CDPRST  =CDPLPL+2       ;"PRESS START"
CD2GAM  =CDLOGP+2   ;"2 GAME MIN"
CDBOXP  =CDPRST+2       ;LOGO BOX
CDLOGP  =CDBOXP+2       ;LOGO ITSELF

DROUTAD:
    .WORD DENORM-1      ;GAME PLAY - TOP OF WELL, DOWN THE TUBE
    .WORD DSPSYS-1      ;SYSTEM CONFIGURATION
    .WORD DSBOOM-1      ;GAME PLAY - BOOM
    .WORD GETDSP-1      ;DATA ENTRY - HI SCORE INITIALS
    .WORD RQRDSP-1      ;DATA ENTRY - RANKING
    .WORD LDRDSP-1      ;INFO ONLY - HI SCORE TABLE
    .WORD DGOVER-1      ;            GAME OVER PLAYER X
    .WORD DPLPLA-1      ;            PLAY PLAYER X
    .WORD DPRSTA-1      ;"PRESS START"
    .WORD BOXPRO-1      ;LOGO BOX
    .WORD LOGPRO-1      ;LOGO
    .WORD D2GAME-1      ;2 GAME MINIMUM
*/
// ;DISPLAY STATE
#define QDSTATE 0x0001 


inline unsigned char getTempestByte(int adr)
{
  // direct read, not triggering any emulation thingy
  return mem[adr].cell;
}
  

void tempestCallback(int type)
{
  if (type == 999) // openPage callback
  {
    // int gameState = getTempestByte(QSTATE); // status bit 7 not set
    
    // adjust emulation / display parameters
    // according to game state
    
    // eg respect "many" dots in "BOOM"
    
    return;
  }
  
  int attractMode = ((getTempestByte(QSTATUS) & 0x80) == 0); // status bit 7 not set


  if (attractMode) return; // no sound in attract mode
/*  
  switch (type)
  {
    case TEMP_SOUND_LAUNCH_PLAYER_FIRE: {v_playSFXStart(playerShot_alt1_data, 0, 0);break;}
    case TEMP_SOUND_CURSOR_MOVES: {v_playSFXStart(cursorMove_alt1_data, 1, 0);break;}


    case TEMP_SOUND_PLAYER_DIES: {break;}
    case TEMP_SOUND_SCORE: {break;}
    case TEMP_SOUND_ENEMY_SHOT: {break;}
    case TEMP_SOUND_ENEMY_EXPLOSTION: {break;}
    case TEMP_SOUND_THRUST_ON_TUBE: {break;}
    case TEMP_SOUND_THRUST_ON_SPACE: {break;}
    case TEMP_SOUND_LINE_DESTRUCTION: {break;}
    case TEMP_SOUND_SLAM: {break;}
    case TEMP_SOUND_3S_WARNING: {break;}
    case TEMP_SOUND_PUKSATION_ON: {break;}
    case TEMP_SOUND_PUKSATION_OFF: {break;}
    default:
      break;
  }
*/
}    

int tempest(void)
{
  ProgName = "tempest"; // defined in display.h
  gameCallback = tempestCallback;
  
  mem = (elem *) gameMemory;
  game = pick_game (ProgName);

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

  init_graphics();
  setAppFPS(60);
  setup_game();
  void audio_set_callback(audio_sample_callback_t cb, void *userdata);
  //void callbackPokey(void *userdata, uint8_t *stream, int length);
  void callbackPokey(void *userdata, int16_t *stream, int length);
  audio_set_callback(callbackPokey, NULL);
	esp_add_event_listener(esp_event_listener);

  g_color_mode = 1;
  yuv_palette_init();
  // from pokey.c
  //  void enablePokeyOutput(int e);
  //  enablePokeyOutput(0); // false
  
  extern int PC;
  extern int A;
  extern int X;
  extern int Y;
  extern long totcycles;
  extern int CC;

  PC = (memrd(0xfffd,0,0) << 8) | memrd(0xfffc,0,0);
  A = 0;
  X = 0;
  Y = 0;
  byte_to_flags (0);

  totcycles = 0;
  irq_cycle = 4173; // tempest 
  CC=0; SET_I; 


  avg_reset_really();
  tobekilled = 0;








	while (1)
	{
    uint64_t start = esp_timer_get_time();
    sim_6502 (25200); // 1512000 Mhz / 60 FPS
		//readKeyEvents(); // single key test makes it impractical to do as an event
		if (tobekilled) break;

      // slow down if we are too fast
      // emulating 30000 vectrex cycles should take 1/50 of a second...
      // if MAX_EMU_FPS is 50 and 50 is reached, we run with 100% original speed
      uint64_t now = esp_timer_get_time();
      if (now - start <= g_fpsToReach) 
      {
          int delay = g_fpsToReach - (now - start);
      //    esp_rom_delay_us(delay);  
      }
	}

  audio_set_callback(NULL, NULL);
  callbackPokey(NULL, NULL, 0);
	esp_add_event_listener(esp_event_listener);
  esp_remove_event_listener(esp_event_listener);

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

