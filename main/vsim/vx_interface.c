#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "../defines.h"
#include "game.h"


// choosing a fixed scale here and adjusting per game in draw_line.
// this scale lets us draw with a 1:1 aspect ratio, ie drawing a
// square 100 units on a side actually looks like a square.

// may need some adjustment for clipping off-screen in the vecterm package


/*
 * no_interface.c: null display for Atari Vector game simulator
 *
 * Copyright 1993, 1996 Eric Smith
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
 * $Id: vx_interface.c,v 1.1 2018/07/31 01:19:45 pi Exp $
 */


#include "display.h"
#include "memory.h"


int smallwindow;
int window_width, window_height;
int stramash_config1=0;
int stramash_config2=0;
int yates_config = 0;
int onlyOnejoystick = 1;
char *ProgName = "NameUnset";

//#define SETTINGS_SIZE 1024
//unsigned char settingsBlob[SETTINGS_SIZE];

void gameCommand(void)
{
	printf("Atari Vector game simulator, Copyright 1993, 1996 Eric Smith\r\n");
}


int OFFSET_X= 500;
int OFFSET_Y= 300;
int MUL_X= 35;
int MUL_Y= 40;


void term_graphics (void)
{
}

#define uses_analog_joystick(x) (((x) == LUNAR_LANDER) || ((x) == RED_BARON)|| ((x) == TEMPEST))


void (*customInputHandler) (void) = 0;

void handle_input (void) // call once per frame
{
#define JOYSTICK_CENTER_MARGIN 20
  if (uses_analog_joystick(game))
  {
	// build a "0" zone

    joystick.x = g_inputState.j0_x;  // 0x80   0  0x7f
    if ((g_inputState.j0_x>0) && (g_inputState.j0_x<JOYSTICK_CENTER_MARGIN)) joystick.x= 0;
    if ((g_inputState.j0_x<0) && (g_inputState.j0_x>-JOYSTICK_CENTER_MARGIN)) joystick.x=  0;
    joystick.y = g_inputState.j0_y;  // 0x80   0  0x7f
    if ((g_inputState.j0_y>0) && (g_inputState.j0_y<JOYSTICK_CENTER_MARGIN)) joystick.y=  0;
    if ((g_inputState.j0_y<0) && (g_inputState.j0_y>-JOYSTICK_CENTER_MARGIN)) joystick.y=  0;

  }
  else
  {
	// build a "0" zone

    joystick.x = g_inputState.j0_x;  // 0x80   0  0x7f
    if ((g_inputState.j0_x>0) && (g_inputState.j0_x<JOYSTICK_CENTER_MARGIN)) joystick.x= 0;
    if ((g_inputState.j0_x<0) && (g_inputState.j0_x>-JOYSTICK_CENTER_MARGIN)) joystick.x=  0;
    joystick.y = g_inputState.j0_y;  // 0x80   0  0x7f
    if ((g_inputState.j0_y>0) && (g_inputState.j0_y<JOYSTICK_CENTER_MARGIN)) joystick.y=  0;
    if ((g_inputState.j0_y<0) && (g_inputState.j0_y>-JOYSTICK_CENTER_MARGIN)) joystick.y=  0;
  }

  start1 = ((g_inputState.buttonState&1)==0);  // button 1 on port 1
  start2 = (g_inputState.buttonState&2)==0;  // button 2 on port 1

  if ((g_inputState.buttonState & 0x06) == 0x00) // button 2+3 together
  {
    cslot_left=1;
//    cslot_right=1;
//    cslot_util=1;

//    slam++; // what the heck is "slam"
  }
  else
  {
    cslot_left=0;
  }
  self_test = 0; // not implemented
  slam = 0; //not implemented
  
  if (game == BATTLEZONE)
    {
      if (stramash_config1)
      {
        switches [0].leftfwd = (g_inputState.j0_y>JOYSTICK_CENTER_MARGIN);
        switches [0].leftrev = (g_inputState.j0_y<-JOYSTICK_CENTER_MARGIN);
        switches [0].rightfwd = ((g_inputState.buttonState&0x01)==0)?1:0; // button 1 port 1
        switches [0].rightrev = ((g_inputState.buttonState&0x02)==0)?1:0; // button 2 port 1
        switches [0].fire = ((g_inputState.buttonState&0x04)==0)?1:0; // button 3 port 1
        switches [0].fire = ((g_inputState.buttonState&0x08)==0)?1:0; // button 4 port 1
      }
      else if (stramash_config2)
      {
        switches [0].leftfwd = (g_inputState.j1_y>JOYSTICK_CENTER_MARGIN);
        switches [0].leftrev = (g_inputState.j1_y<-JOYSTICK_CENTER_MARGIN);
        switches [0].rightfwd = ((g_inputState.buttonState&0x10)==0)?1:0; // button 1 port 1
        switches [0].rightrev = ((g_inputState.buttonState&0x20)==0)?1:0; // button 2 port 1
        switches [0].fire = ((g_inputState.buttonState&0x40)==0)?1:0; // button 3 port 1
        switches [0].fire = ((g_inputState.buttonState&0x80)==0)?1:0; // button 4 port 1
      }
      else if (yates_config)
      {
        switches [0].leftfwd = ((g_inputState.buttonState&0x10)==0)?1:0; // button 1 port 2
        switches [0].leftrev = ((g_inputState.buttonState&0x20)==0)?1:0; // button 2 port 2
        switches [0].rightfwd = ((g_inputState.buttonState&0x40)==0)?1:0; // button 3 port 2
        switches [0].rightrev = ((g_inputState.buttonState&0x80)==0)?1:0; // button 4 port 2
        switches [0].fire = g_inputState.j1_x>JOYSTICK_CENTER_MARGIN;  //Joy2 analog x axis pulled high      
      }
      else if (onlyOnejoystick)
      {
        switches [0].fire = ((g_inputState.buttonState&0x08)==0)?1:0; // button 4 port 1
        switches [0].leftfwd = (g_inputState.j0_y>JOYSTICK_CENTER_MARGIN) || (g_inputState.j0_x>JOYSTICK_CENTER_MARGIN);
        switches [0].leftrev = (g_inputState.j0_y<-JOYSTICK_CENTER_MARGIN) || (g_inputState.j0_x<-JOYSTICK_CENTER_MARGIN);
        switches [0].rightfwd = (g_inputState.j0_y>JOYSTICK_CENTER_MARGIN) || (g_inputState.j0_x<-JOYSTICK_CENTER_MARGIN);
        switches [0].rightrev = g_inputState.j0_y<-JOYSTICK_CENTER_MARGIN  || (g_inputState.j0_x>JOYSTICK_CENTER_MARGIN);


          if ((g_inputState.j0_x>JOYSTICK_CENTER_MARGIN) && (g_inputState.j0_y>JOYSTICK_CENTER_MARGIN))
          {
              switches [0].leftfwd = 1;
              switches [0].leftrev = 0;
              switches [0].rightfwd = 0;
              switches [0].rightrev = 0;
          }
          if ((g_inputState.j0_x>JOYSTICK_CENTER_MARGIN) && (g_inputState.j0_y<JOYSTICK_CENTER_MARGIN))
          {
              switches [0].leftfwd = 0;
              switches [0].leftrev = 0;
              switches [0].rightfwd = 0;
              switches [0].rightrev = 1;
          }
          if ((g_inputState.j0_x<-JOYSTICK_CENTER_MARGIN) && (g_inputState.j0_y>JOYSTICK_CENTER_MARGIN))
          {
              switches [0].leftfwd = 0;
              switches [0].leftrev = 0;
              switches [0].rightfwd = 1;
              switches [0].rightrev = 0;
          }
          if ((g_inputState.j0_x<-JOYSTICK_CENTER_MARGIN) && (g_inputState.j0_y<JOYSTICK_CENTER_MARGIN))
          {
              switches [0].leftfwd = 0;
              switches [0].leftrev = 1;
              switches [0].rightfwd = 0;
              switches [0].rightrev = 0;
          }
     
      }
      else
      {
        switches [0].leftfwd = g_inputState.j0_y>JOYSTICK_CENTER_MARGIN;
        switches [0].leftrev = g_inputState.j0_y<-JOYSTICK_CENTER_MARGIN;
        switches [0].rightfwd = g_inputState.j1_y>JOYSTICK_CENTER_MARGIN;
        switches [0].rightrev = g_inputState.j1_y<-JOYSTICK_CENTER_MARGIN;
        switches [0].fire = ((g_inputState.buttonState&0x08)==0)?1:0; // button 4 port 1
      }
      
    }
  else if (game == RED_BARON)
  {
      // center Joy is 0x80
      // max Joy is 0xc0
      // min Joy is 0x40
      signed char y = (signed char) ((signed char)joystick.y)/2;
      signed char x = (signed char) ((signed char)joystick.x)/2;
      joystick.y = 128 + y;
      joystick.x = 128 + x;
    
      switches [0].fire = ((g_inputState.buttonState&0x08)==0)?1:0; // button 4 port 1
  }
  else if (game == LUNAR_LANDER)
  {
      switches [0].left = g_inputState.j0_x<-JOYSTICK_CENTER_MARGIN;
      switches [0].right = g_inputState.j0_x>JOYSTICK_CENTER_MARGIN;
      if (((signed char)joystick.y)<0) joystick.y = 0;

      
      // it is IMPORTANT
      // to hit and relase after a time
      // since Lunar Lander "samples" the button state for five rounds
      // only if button is pressed long enough
      // the coin counter will accept it
      if ((g_inputState.buttonState&0x06)==0)
        {
          cslot_right=1;
        }
        else
        {
          cslot_right=0;
        }
  }
  else  if (game == BLACK_WIDOW)
  {
#define bw_up    left
#define bw_down  right
#define bw_left  shield
#define bw_right fire
      // MOVE
      switches [0].bw_up = g_inputState.j0_y>30; // UP
      switches [0].bw_down = g_inputState.j0_y<-30; // DOWN
      switches [0].bw_right =  g_inputState.j0_x<-30;// RIGHT
      switches [0].thrust = (((g_inputState.buttonState&4)==0))?1:0; // not used
      switches [0].hyper = (((g_inputState.buttonState&2)==0))?1:0; //not used
      switches [0].bw_left = g_inputState.j0_x>30; // LEFT for BW
      switches [0].abort = 0; // no input

      // FIRE
      switches [1].bw_up = g_inputState.j1_y>30;
      switches [1].bw_down = g_inputState.j1_y<-30;
      switches [1].bw_right = g_inputState.j1_x<-30; 
      switches [1].thrust = (((g_inputState.buttonState&0x40)==0))?1:0; // not used
      switches [1].hyper = (((g_inputState.buttonState&0x20)==0))?1:0; // not used
      switches [1].bw_left = g_inputState.j1_x>30;
      switches [1].abort = 0; // no input
 
      // also uses 
      //  start1 = currentButtonState & 0x01;  // button 1 on port 1
      //  start2 = currentButtonState & 0x02;  // button 2 on port 1
      
    }
    else
    {
      switches [0].left = g_inputState.j0_x<-JOYSTICK_CENTER_MARGIN;
      switches [0].right = g_inputState.j0_x>JOYSTICK_CENTER_MARGIN;
      switches [0].fire = (((g_inputState.buttonState&0x08)==0))?1:0; // button 4 port 1
      switches [0].thrust = (((g_inputState.buttonState&0x04)==0))?1:0; // button 3 port 1
      switches [0].hyper = (((g_inputState.buttonState&0x02)==0))?1:0; // button 2 port 1
      switches [0].shield = (((g_inputState.buttonState&0x01)==0))?1:0; // button 1 port 1
      switches [0].abort = 0; // no input
      
      switches [1].left = g_inputState.j1_x<-JOYSTICK_CENTER_MARGIN;
      switches [1].right = g_inputState.j1_x>JOYSTICK_CENTER_MARGIN;
      switches [1].fire = (((g_inputState.buttonState&0x80)==0))?1:0; // button 4 port 2
      switches [1].thrust = (((g_inputState.buttonState&0x40)==0))?1:0; // button 3 port 2
      switches [1].hyper = (((g_inputState.buttonState&0x20)==0))?1:0; // button 2 port 2
      switches [1].shield = (((g_inputState.buttonState&0x10)==0))?1:0; // button 1 port 2
      switches [1].abort = 0; // no input
    }
}



  


void mini_draw_line_color(int x0, int y0, int x1, int y1, int color, uint8_t brightness); // brightness or color - depending on mode, 0 is always undraw
void mini_draw_line(int x0, int y0, int x1, int y1, uint8_t brightness); // brightness or color - depending on mode, 0 is always undraw
void mini_end_frame(void);

//; MX AVG - MIN AVG for a game

DRAM_ATTR int ALG_XMIN=0;
DRAM_ATTR int ALG_XMAX=1000;
DRAM_ATTR int ALG_YMIN=0;
DRAM_ATTR int ALG_YMAX=750;
DRAM_ATTR int ALG_MAX_X=1000; // wid
DRAM_ATTR int ALG_MAX_Y=750;

DRAM_ATTR int POS_ADDER_X = 0;
DRAM_ATTR int POS_ADDER_Y = 0;
DRAM_ATTR int FLIP_Y = 0;
DRAM_ATTR int FLIP_X = 0;
DRAM_ATTR int SCREEN_TYPE = GAME_LANDSCAPE;



DRAM_ATTR static float scl_factorx; // for some unkown reason these cannot be uint32_t, if it is uint32_t then lines are sometimes out of bounds
DRAM_ATTR static float scl_factory; // inspite the fact, that the scale factor is NEVER negative!!!
DRAM_ATTR static long offx;
DRAM_ATTR static long offy;


DRAM_ATTR int displayWidth = 0;
DRAM_ATTR int displayHeight = 0;

void init_graphics ()
{
    displayWidth = getDisplayWidth();
    displayHeight = getDisplayHeight();
    int sw = getScreenWidth();
    int sh = getScreenHeight();

    // screen type is that of the game/ emulator
    if (SCREEN_TYPE == GAME_PORTRAIT)
    {
      if (sh>sw) // display portait
      {
          // sw is the factor that stays
          displayHeight =(int) (((float)sw)*4.0f/3.0f);
          displayWidth = sw;

          scl_factorx = ((float)ALG_MAX_X) / ((float)displayWidth);
          scl_factory = ((float)ALG_MAX_Y) / ((float)displayHeight);
          offx = (sw-displayWidth)/2;
          offy = (sh-displayHeight)/2;

          if (game == TEMPEST)
          {
              displayWidth = 480;
              displayHeight =(int) (((float)displayWidth)*4.0f/3.0f);

              scl_factorx = ((float)ALG_MAX_X) / ((float)displayWidth);
              scl_factory = ((float)ALG_MAX_Y) / ((float)displayHeight);
              offx = (sw-displayWidth)/2;
              offy = (sh-displayHeight)/2;
              offy-=50;
              offx+=0;
          }
          printf("Portrait portait correction\n");
      }
      else // display is Landscape
      {
          // sh is the factor that stays
          displayWidth =(int) (((float)sh)*3.0f/4.0f);
          displayHeight = sh;
          scl_factorx = ((float)ALG_MAX_X) / ((float)displayWidth);
          scl_factory = ((float)ALG_MAX_Y) / ((float)displayHeight);
          offx = (sw-displayWidth)/2;
          offy = (sh-displayHeight)/2+110;
          printf("Portrait Landscape correction\n");
      }
    }
    else // LANDCSAPE
    {
      if (sw>sh)
      {
          displayHeight = sh-50;
          displayWidth =(int) (((float)displayHeight)*4.0f/3.0f);
          scl_factorx = ((float)ALG_MAX_X) / ((float)displayWidth);
          scl_factory = ((float)ALG_MAX_Y) / ((float)displayHeight);
          offx = (sw-displayWidth)/2+100;
          offy = (sh-displayHeight)/2-110;

          printf("Landscape landscape correction\n");

      }
      else
      {
          // sw is the factor that stays
          displayHeight =(int) (((float)sw)*3.0f/4.0f);
          displayWidth = sw;

          scl_factorx = ((float)ALG_MAX_X) / ((float)displayWidth);
          scl_factory = ((float)ALG_MAX_Y) / ((float)displayHeight);
          offx = (sw-displayWidth)/2+70;
          offy = (sh-displayHeight)/2-70;
        printf("Landscape portait correction\n");
      }
    }
printf("POS_ADDER_X: %i, POS_ADDER_Y: %i\n", POS_ADDER_X, POS_ADDER_Y);
printf("OffsetX: %li, Offsety: %li\n", offx, offy);

    printf("scl_factorx:%f, scl_factory: %f, ",scl_factorx, scl_factory);
}
/* Tempest 4-bit colour → YUV palette index
 *
 * Encoding: %0000rrgb
 *   rr: 00=R0  01=R85  10=R170  11=R255
 *   g:  0=G0   1=G255
 *   b:  0=B0   1=B255
 */
static const uint8_t tempest_color_lut_old[16] = {
    YUV_PALETTE_BLACK,         /*  0: 0000  R=  0, G=  0, B=  0 */
    YUV_PALETTE_BLUE,          /*  1: 0001  R=  0, G=  0, B=128 */
    YUV_PALETTE_BRIGHTGREEN,   /*  2: 0010  R=  0, G=128, B=  0 */
    YUV_PALETTE_CYAN,          /*  3: 0011  R=  0, G=128, B=128 */
    YUV_PALETTE_DARKMAROON,    /*  4: 0100  R= 128, G=  0, B=  0 */
    YUV_PALETTE_BLUEVIOLET,    /*  5: 0101  R= 128, G=  0, B=128 */
    YUV_PALETTE_CHARTREUSE,    /*  6: 0110  R= 128, G=128, B=  0 */
    YUV_PALETTE_AQUAMARINE,    /*  7: 0111  R= 85, G=255, B=255 */
    YUV_PALETTE_DARKRED,       /*  8: 1000  R=170, G=  0, B=  0 */
    YUV_PALETTE_ELECTRICPURP,  /*  9: 1001  R=170, G=  0, B=255 */
    YUV_PALETTE_YELLOWGREEN,   /* 10: 1010  R=170, G=255, B=  0 */
    YUV_PALETTE_PASTELCYAN,    /* 11: 1011  R=170, G=255, B=255 */
    YUV_PALETTE_RED,           /* 12: 1100  R=255, G=  0, B=  0 */
    YUV_PALETTE_MAGENTA,       /* 13: 1101  R=255, G=  0, B=255 */
    YUV_PALETTE_YELLOW,        /* 14: 1110  R=255, G=255, B=  0 */
    YUV_PALETTE_WHITE,         /* 15: 1111  R=255, G=255, B=255 */
};

static const uint8_t tempest_color_lut[16] = {
    YUV_PALETTE_WHITE,        /*  0: raw 0000 -> active-low all-on,  I+R+G+B */
    YUV_PALETTE_TURQUOISE,    /*  1: raw 0001 -> I+R+G     (B off)          */
    YUV_PALETTE_MAGENTA,      /*  2: raw 0010 -> I+R+B     (G off)          */
    YUV_PALETTE_ROYALBLUE,    /*  3: raw 0011 -> I+R       (G,B off)        */
    YUV_PALETTE_LIGHTYELLOW,  /*  4: raw 0100 -> I+G+B     (R off)          */
    YUV_PALETTE_LIGHTGREEN,   /*  5: raw 0101 -> I+G       (R,B off)        */
    YUV_PALETTE_TOMATO,       /*  6: raw 0110 -> I+B       (R,G off)        */
    YUV_PALETTE_GRAY,         /*  7: raw 0111 -> I only    (R,G,B off)      */
    YUV_PALETTE_LIGHTGRAY,    /*  8: raw 1000 -> R+G+B     (I off)          */
    YUV_PALETTE_DARKCYAN,     /*  9: raw 1001 -> R+G       (I,B off)        */
    YUV_PALETTE_VIOLET,       /* 10: raw 1010 -> R+B       (I,G off)        */
    YUV_PALETTE_DARKBLUE,     /* 11: raw 1011 -> R only    (I,G,B off)      */
    YUV_PALETTE_OLIVE,        /* 12: raw 1100 -> G+B       (I,R off)        */
    YUV_PALETTE_DARKGREEN,    /* 13: raw 1101 -> G only    (I,R,B off)      */
    YUV_PALETTE_DARKRED,      /* 14: raw 1110 -> B only    (I,R,G off)      */
    YUV_PALETTE_BLACK,        /* 15: raw 1111 -> nothing lit (all bits off) */
};
IRAM_ATTR void draw_line(int x0, int y0, int x1, int y1, int Colour15, int z) // z is 0:12 in lunar, colour is always 7
{
  if (z == 0) return; // MOVE, possibly to realign at 0,0
    if (g_color_mode)
    {
        mini_draw_line_color( offx + (x0+POS_ADDER_X) / scl_factorx, 
                        offy + (y0+POS_ADDER_Y) / scl_factory, 
                        offx + (x1+POS_ADDER_X) / scl_factorx, 
                        offy + (y1+POS_ADDER_Y) / scl_factory, 
                        tempest_color_lut_old[Colour15],
                        z<<4); // For ESP32
    }
    else
    {
      mini_draw_line( offx + (int) ((x0+POS_ADDER_X) / scl_factorx), 
                      offy + (int) ((y0+POS_ADDER_Y) / scl_factory), 
                      offx + (int) ((x1+POS_ADDER_X) / scl_factorx), 
                      offy + (int) ((y1+POS_ADDER_Y) / scl_factory), 
                      z<<4); // For ESP32
    }
}
// from addpoint AVG (Tempest)
// in color is a 4 bit rgb value
// %0000rrgb
IRAM_ATTR void draw_line2(int x0, int y0, int x1, int y1, int Colour15, int z) // z is 0:12 in lunar, colour is always 7
{
  if (z == 0) return; // MOVE, possibly to realign at 0,0
  
//   if (z>=12) 
   {

      if (game==TEMPEST)
      {

          if (FLIP_Y)
          {
            mini_draw_line_color(          offx + (x0+POS_ADDER_X) / scl_factorx, 
                            displayHeight-(offy + (y0+POS_ADDER_Y) / scl_factory), 
                                           offx + (x1+POS_ADDER_X) / scl_factorx, 
                            displayHeight-(offy + (y1+POS_ADDER_Y) / scl_factory), 
                            tempest_color_lut_old[Colour15],
                            z<<4); // For ESP32
          }
          else
          {
            //printf("A Sim Line col:x0:%i, y0:%i, x1:%i, y1:%i, col:%i, b:%i\n",x0,y0,x1,y1,Colour15,  z);
            printf("Draw Line 1\n");
            mini_draw_line_color( offx + (x0+POS_ADDER_X) / scl_factorx, 
                                  offy + (y0+POS_ADDER_Y) / scl_factory, 
                                  offx + (x1+POS_ADDER_X) / scl_factorx, 
                                  offy + (y1+POS_ADDER_Y) / scl_factory, 
                            tempest_color_lut_old[Colour15],
                            z<<4); // For ESP32
          }
      }
      else
      {
            if (FLIP_Y)
            {
              mini_draw_line_color(          offx + (x0+POS_ADDER_X) / scl_factorx, 
                              displayHeight-(offy + (y0+POS_ADDER_Y) / scl_factory), 
                                             offx + (x1+POS_ADDER_X) / scl_factorx, 
                              displayHeight-(offy + (y1+POS_ADDER_Y) / scl_factory), 
                              tempest_color_lut[Colour15],
                              z<<4); // For ESP32
            }
            else
            {
              printf("B Sim Line col:x0:%i, y0:%i, x1:%i, y1:%i, col:%i, b:%i\n",x0,y0,x1,y1,Colour15,  z);
              mini_draw_line_color( offx + (x0+POS_ADDER_X) / scl_factorx, 
                                    offy + (y0+POS_ADDER_Y) / scl_factory, 
                                    offx + (x1+POS_ADDER_X) / scl_factorx, 
                                    offy + (y1+POS_ADDER_Y) / scl_factory, 
                              tempest_color_lut[Colour15],
                              z<<4); // For ESP32
            }
      }
   }
  
}

void open_page (int step)
{
}


void close_page (void)
{
  // code executes faster than realtime, slow it down here to 50hz
  // (though 60hz would be correct speed)
  // OR WAIT FOR VECTREX 1/50Hz HERE?
  if (gameCallback != 0) gameCallback(999);
  handle_input();
  mini_end_frame();
}


int repeat_frame (void)
{
  return (false);
}


void bell (void)
{
}
