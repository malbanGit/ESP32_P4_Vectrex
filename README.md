To build:
Download Visual Studio Code
Install the extension ESP-IDF
I used version ESP-IDF V5.5.1.

And build. I connected my ESP development board with a simple USB cable and flashed it with UART.

[![Video ansehen](https://img.youtube.com/vi/WvN1xyKDec0/0.jpg)](https://youtu.be/WvN1xyKDec0)



Final report: Vectrex emulator on an ESP32 P4

Of course, I am not THE authority in programming an ESP32. In fact, I only started a few weeks ago.

I estimate that I spent around 80–100 hours learning — and I do not consider that a waste of time.
Very briefly summarized — from my experience, the ESP32 P4 is too slow to reasonably emulate the Vectrex.

Explanations:

My hardware (from Amazon):
(https://www.amazon.de/Waveshare-ESP32-P4-Module-DEV-KIT-C-ESP32-P4-Module-High-Performance-Development/dp/B0F2FCWCJ8
)

Waveshare ESP32-P4-Module-DEV-KIT-C
ESP32-P4-Module High-Performance Development Board
Based on ESP32-P4 & ESP32-C6
Supports Wi-Fi 6 & BT5/BLE
With speaker & RPi Camera (B) & 10.1” DSI touchscreen

The ESP32 P4 is equipped with a CPU that has two cores and currently runs at 360 MHz — this is expected to increase to 400 MHz in the future.
There is no GPU — anything that gets drawn must be done by the CPU itself.
However, there is CPU-independent DMA transfer to the LCD, which helps a bit.
The drawback is that the DMA still uses the same memory subsystem and the same access bus.
This means that “locks” and “waits” happen very quickly and very often when accessing the same memory — which is exactly the case when drawing graphics into frame memory.

The display resolution is 1280×800, which in RGB565 format corresponds to 2,048,000 bytes = 2 MB.
Which means framebuffers do not fit in "fast" RAM only in external PSRAM.

The processor / dev-board provides different memory types:
- Flash / 16 MB rather slow
- DRAM on-chip SRAM 768 KB very very fast (which is divided in IRAM (instructions) and DRAM (data)
- PSRAM 32 MB slow “external” RAM

I slowly worked my way deeper into the ESP32 and its hardware.

1) LVGL — Line widget
I installed ESP-IDF (5.5.1) in Microsoft Visual Studio Code and eventually was able to start coding.
You have to get used to the IDE as well (how to set things, where to look, etc.).
I did not find truly good documentation — it was a lot of trial and error.

Eventually I managed to create a working project and display something on the screen.

I took the demo “Advanced/lvgl_demo_v9” as a starting point and began experimenting with the LVGL graphics library.

The rendering speed was terrible — between 1–4 FPS.
The problem is that LVGL is intended for widget-based UI elements that represent “self-contained” graphical components.
Drawing hundreds of lines creates enormous overhead.

2) LVGL — vectors
In the second attempt, I still used LVGL, but abandoned the widgets.
The library offers more hardware-direct vector functions, so-called “vector drawing" .
You can submit paths consisting of moves and draws — in theory that sounds good…
but in practice it's not. Slightly faster than the line widget — but still much too slow.

3) LVGL with direct framebuffer access
Briefly tested.
In LVGL this is a strange construct providing a “canvas” framebuffer.
It is not truly direct, and double buffering is also not possible.
Still too slow.
Screen clearing or line erasing works poorly or extremely slow.

4) Raw framebuffer access
I abandoned LVGL and tried accessing the framebuffer directly using ESP-specific functions.
This was also the first time I integrated the original VECX emulator to get realistic behavior.

For this purpose, I run the emulator on Core 1 and the renderer on Core 0.
Communication happens via a queue. The emulator tells the renderer when to draw a line — and when a frame is done.
VSync synchronizes everything to avoid tearing.
Everything runs on a single framebuffer.
Clearing the screen lines still consumes a lot of time.
But the emulator runs and draws. Approx. 8 FPS.

5) Raw framebuffer — software double buffering
Due to a misunderstanding of how a function worked, I could not get a second hardware framebuffer.
Thus I implemented a software framebuffer — slow and frustrating. (fought with it for a long while)

6) Custom display driver
I created my own display driver package.
A mix of Waveshare and Espressif components, hoping to request a real second hardware framebuffer.
After many adjustments — and understanding how ESP-IDF dependencies work — I succeeded.
Not much faster, but a little!

For clarification:
A DrawLine routine is very “old school”.
There is a draw_pixel() routine — and the DrawLine method calls it for every single pixel.
I implemented different DrawLine methods: a simple Bresenham, one for thicker lines, one with anti-aliasing.
Adjustable with global variables.

7) Custom driver with real double buffering
After finally finding the earlier (ChatGPT’s) bug after long debugging — I now have two real hardware framebuffers that I can swap between.
There are still issues with efficiently clearing the inactive framebuffer — still around ~10–11 FPS.

8) Custom driver — real double buffering — flicker-free
Achieved: the display is flicker-free.
The emulator task and renderer task are now logically separated.
The emulator tells the renderer after 30000 cycles: “hey I’m done”.
Then the renderer swaps the display to the backbuffer that has recently been drawn to.
There are now two FPS displays:
One for the renderer, which should ideally operate at 60Hz (interestingly later stabilizes around 70??)
And one emulator FPS, which ideally should reach at least 50.
Currently still showing ~20FPS and 20FPS.

9) Vectrex improvements
I ported the LibRetro version to the ESP.
But I discovered that this version is 2× slower then before.
This is due to precise drift emulation and capacitor simulation.
Both use double variables. The ESP32 (32-bit!) hates double (64-bit) — it triggers software floating point emulation which slows EVERYTHING down.

10) Rollback and optimization attempts — final
I removed the doubles again. The emulation now runs mostly correctly.
(Sound, input, ROM loading still untouched)
I found another bug in FPS display and core separation.
FPS indicators are now no longer synchronized artificially — but correctly.

The renderer — with several optimizations — now truly manages to deliver one image per VSync.
And also clear the old framebuffer to blank!
This results in a sharp image, no jitter, and no graphical artifacts.

Thus, the renderer FPS is maxed out at ~69 FPS — even though it should be VSync-limited at 60.

BUT!
The emulator core does not achieve more than ~17–19 FPS.
I tried everything I know.
Compiler at -O3...
Inlining...
All data in DRAM, all code in IRAM (fast RAM).
Auto-sync precisely aligned to frame timing.

I do not know how to make the emulator faster.
Initially I thought graphics were the bottleneck.
But in fact it is the pure CPU performance at 360 MHz that is insufficient to run the emulator at real-time speed.

At least not a vecx derivat written in C. Assembler might be different — but attempting that would be madness.

Maybe there are smarter ESP32 programmers out there — but I have reached the end of what I know — and must now give up.
