# Vectrex Mini — ESP32-P4 Vector Game Framework

A real-time vector-graphics emulation and rendering framework for the **ESP32-P4** microcontroller, capable of running Vectrex and Atari vector-arcade emulators on a bare-metal dual-core RISC-V chip — no GPU, no OS, no frame-clearing.

> **Status:** Alpha — based on the ESP32-P4 (non-X silicon). Tested with ESP-IDF 5.5.1.  
> ESP-IDF 6.x is **not** compatible with this chip revision.

---

## Table of Contents

1. [Hardware](#hardware)
2. [Architecture Overview](#architecture-overview)
3. [Core 0 — Renderer](#core-0--renderer)
4. [Core 1 — Application](#core-1--application)
5. [Frame Pipeline](#frame-pipeline)
6. [Line Drawing](#line-drawing)
7. [Color System — YUV422](#color-system--yuv422)
8. [Overlay System](#overlay-system)
9. [Audio](#audio)
10. [Input](#input)
11. [Events](#events)
12. [Writing Your Own Application](#writing-your-own-application)
13. [API Reference](#api-reference)
14. [Global Tuning Parameters](#global-tuning-parameters)
15. [Key Bindings](#key-bindings)
16. [Directory Structure](#directory-structure)
17. [Build](#build)

---

## Hardware

| Component | Details |
|---|---|
| SoC | Espressif ESP32-P4 (alpha, non-X), dual-core RISC-V @ 360 MHz |
| RAM | 32 MB PSRAM (200 MHz bus) + 768 kB DRAM |
| LCD | 480 × 800, used in **portrait** (vertical) orientation |
| HDMI output | 1280 × 720 via LT8912B bridge chip |
| Color format | **YUV422** throughout (`VIDEO_FB_YUV422`) |
| Audio | ES8311 codec · I²S · 44100 Hz · 16-bit stereo |
| Storage | SD card (FAT, for ROM files and overlay PNGs) |
| USB | HID keyboard input (polled, not interrupt-driven) |

### Why YUV422?

The frame buffer for 1280 × 720 in RGB888 would require **2.76 MB per frame** and push the PSRAM bus beyond its limits at 60 Hz. YUV422 uses **2 bytes per pixel** (YUYV packing: `[Y0][U][Y1][V]` per 4-byte pair), halving the bandwidth. Two hardware frame buffers live in PSRAM; black pixels are initialised to `0x80008000` (Y = 0, neutral chroma U = V = 128).

An earlier RGB888 code path exists in the repository for reference but is no longer used.

---

## Architecture Overview

```
┌──────────────────────────────────────────────────────────────────┐
│                        ESP32-P4 Dual-Core                        │
│                                                                  │
│  ┌──────────────────────────┐  ┌──────────────────────────────┐  │
│  │        CORE 0            │  │           CORE 1             │  │
│  │      "Renderer"          │  │        "Application"         │  │
│  │      priority 3          │  │         priority 7           │  │
│  │                          │  │                              │  │
│  │  wait VSYNC semaphore    │  │  while(1) { my_app(); }      │  │
│  │        │                 │  │        │                     │  │
│  │  pick newest READY frame │  │  emulate / compute           │  │
│  │        │                 │  │        │                     │  │
│  │  diff vs previous frame  │  │  mini_draw_line*(...)        │  │
│  │  (stable-line algorithm) │  │        │                     │  │
│  │        │                 │  │  mini_end_frame()            │  │
│  │  undraw removed lines    │  │        │                     │  │
│  │  draw   new    lines     │  │  ── marks slot READY ──►     │  │
│  │        │                 │  │                              │  │
│  │  swap front/back buffer  │  │  ┌─────────────────────┐     │  │
│  │  esp_lcd_draw_bitmap     │  │  │   audio_music_task  │     │  │
│  └──────────────────────────┘  │  │   priority 20       │     │  │
│                                │  │   calls audio CB    │     │  │
│  ┌──────────────────────────┐  │  │   → I²S / codec     │     │  │
│  │     VSYNC ISR (IRAM)     │  │  └─────────────────────┘     │  │
│  │  xSemaphoreGiveFromISR   │  └──────────────────────────────┘  │
│  └──────────────────────────┘                                    │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │                Triple-Buffered Frame Slots                 │  │
│  │   slot[0]  slot[1]  slot[2]   (up to 1000 lines each)      │  │
│  │   FREE → BUILDING → READY → RENDERING → FREE               │  │
│  └────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

The **framework** (Core 0 + the frame pipeline) and the **application** (Core 1) are completely decoupled. The framework has no include, no variable, and no direct knowledge of what the application does. The only coupling is:

* The application calls `mini_draw_line*(...)` and `mini_end_frame()` to submit a frame.
* The framework fires `ESP_EVENT_*` events that the application may choose to handle.

---

## Core 0 — Renderer

Core 0 runs `renderer_task` (priority 3) and does **nothing but render**.

```
VSYNC IRQ fires
      │
      ▼
xSemaphoreGive (ISR-safe)
      │
      ▼  (renderer wakes)
find newest FRAME_READY slot
      │
      ▼
stable-line diff algorithm
  ┌─ for each line in previous frame: hash-table lookup in new frame ─┐
  │  if gone    → undraw_line(old position)                           │
  │  if present → nothing (already on screen)                         │
  │  if new     → draw_line(new position)                             │
  └───────────────────────────────────────────────────────────────────┘
      │
      ▼
swap front ↔ back buffer
esp_lcd_panel_draw_bitmap  (DMA → LT8912B → LCD / HDMI)
      │
      ▼
slot → FRAME_FREE
```

**Why no `clearScreen`?**  
Clearing a 1280 × 720 YUV422 frame buffer takes ~1.8 MB of writes per frame. At 60 Hz and a 200 MHz PSRAM bus that is simply not possible while also running the emulator. Instead, each line is individually *undrawn* by restoring the pixels it touched. The stable-line diff ensures lines that did not change are never redrawn at all.

---

## Core 1 — Application

Core 1 runs `application_task` (priority 7):

```c
IRAM_ATTR static void application_task(void *arg)
{
    while (1)
    {
        int vectrex(void);
        vectrex();
    }
}
```

This is the **only** thing the framework knows about the application. No header is included; no symbol is shared beyond the one function name. The application is responsible for registering itself, running its own loop, and returning cleanly when it receives `ESP_EVENT_KILL`.

`audio_music_task` also runs on Core 1 at priority 20. Because it blocks on codec I/O (~20 ms per 882-sample buffer at 44100 Hz / 50 fps), it does not significantly starve the application task.

---

## Frame Pipeline

Three frame slots are maintained in DRAM, each holding up to **1000 line descriptors**.

```
State machine per slot:

  FRAME_FREE
      │  application calls mini_draw_line*(...)
      ▼
  FRAME_BUILDING          ← application writes lines here
      │  application calls mini_end_frame()
      ▼
  FRAME_READY             ← renderer may pick this up
      │  renderer selects newest READY slot
      ▼
  FRAME_RENDERING         ← renderer is drawing
      │  render complete
      ▼
  FRAME_FREE
```

If the application produces frames faster than the renderer can display them, the oldest READY slot is silently overwritten. If the renderer is faster than the application, it re-renders the last frame.

`mini_end_frame()` also calls `readevents()` internally, so input state is refreshed once per application frame without any extra work from the application.

---

## Line Drawing

All drawing goes through one of three **variant families**, each with a C reference implementation and an optimised **RISC-V assembly** version. The assembly variants (`.S` files) run in IRAM and are approximately **10% faster** than the C versions; they are interchangeable algorithmically.

### Variant Families

| Family | Public API | Description |
|---|---|---|
| **Brightness** | `mini_draw_line()` | Greyscale / white. Writes only the Y (luma) channel. |
| **Color** | `mini_draw_line_color()` | Fixed palette of 127 colours. Writes Y + U/V chroma. |
| **Overlay** | (automatic, via `mini_draw_line`) | Additiv to overlay as Brightness on draw; restores overlay background pixels on undraw. |

For each draw function there is a matching internal `undraw` function. The renderer calls undraw automatically for lines that no longer appear in the new frame — the application never needs to manage this.

### Line Properties

Every line has three visual dimensions, all of which can cause the rendered stroke to occupy more than one pixel column/row:

| Property | Global variable | Description |
|---|---|---|
| **Width** | `g_line_width` | Physical stroke width in pixels |
| **Glow** | `g_line_glow` | Soft halo extending beyond the stroke |
| **Brightness** | `brightnessAdjust` | Additive luma boost; high values spill to surrounding pixels via a Gaussian LUT (`gauss_lut`) |

> **Important:** After changing any of these values, always call `changeGlobalLineValues()`.  
> This recalculates derived values (`g_line_Rb`, `g_beam_r`, `g_beam_r2_q16`, `g_gs`) that the assembly routines read directly from globals.

### Display Rotation

* **LVDS / LCD (portrait):** `mini_draw_line` rotates coordinates 90° clockwise before storing: `x′ = LCD_H_RES − y`, `y′ = x`. The application always works in logical portrait coordinates.
* **HDMI:** No rotation. Logical coordinates map 1 : 1 to screen pixels.

---

## Color System — YUV422

`defines.h` declares a 128-entry named palette (`YUV_PALETTE_*` constants, indices 0–127) covering neutrals, primaries, pastels, neons, and earth tones. The actual YUV values are held in the array `s_overlay_palette_yuv[128][3]`.

Initialise the palette once at startup:

```c
yuv_palette_init();
```

Then pass a palette index to `mini_draw_line_color()`:

```c
mini_draw_line_color(x0, y0, x1, y1, YUV_PALETTE_GREEN, brightness);
```

BT.601 full-range conversion is used throughout. YUYV byte order: `[Y0][U][Y1][V]` per 4-byte pair.

---

## Overlay System

An overlay is a **PNG image** loaded from SD card. It provides the coloured background art visible through and around the vector lines, simulating the plastic colour overlays placed over real Vectrex screens.

```c
loadOverlayRGB("/sdcard/SPIKE.png", width, height);
```

### How it works internally

1. The PNG is decoded by **lodepng** into a BGRA PSRAM buffer.
2. For LCD (portrait), the image is rotated 90° CW in software during load.
3. A **127-entry BGR palette** is built from the image's colour histogram.
4. A palettised index map (`s_overlay_pal`, 1 byte/pixel) is generated:
   - **Bit 7 = 0** → fully opaque pixel (palette index in bits 6:0)
   - **Bit 7 = 1** → semi-transparent pixel (scaled by `s_overlay_alpha_val`)
5. YUV caches are built: `s_overlay_palette_yuv` (full intensity) and `s_overlay_palette_yuv_ea` (alpha-scaled, used by undraw to restore the background).
6. The overlay is blitted into **both** hardware frame buffers so it is present regardless of which buffer is currently front.

### Transparency

The alpha threshold is discrete — a pixel with alpha = 255 in the PNG is fully opaque; any other alpha value is treated as semi-transparent (or null). A **global semi-transparency level** (`s_overlay_alpha_val`, default `GLOBAL_OVERLAY_ALPHA = 50`) is applied uniformly to all semi-transparent pixels. Using a fixed global alpha allows the per-pixel blend tables to be precomputed, saving runtime cost.

### Overlay and colour drawing

`mini_draw_line()` is **overlay-aware**:
* If an overlay is loaded, lines are drawn using the overlay's palette colours at the pixels they cross.
* If no overlay is loaded, lines fall back to greyscale.

`mini_draw_line_color()` always uses the fixed YUV palette — it does **not** interact with the overlay. You cannot draw palette-coloured lines on top of an overlay simultaneously.

Toggle the overlay at runtime with the **SPACE** key, or directly via `overlayEnabled = 0` / `1`.

---

## Audio

```
audio_music_task  (Core 1, priority 20)
      │
      ▼
s_audio_cb(userdata, int16_t *stream, length)   ← application provides this
      │
      ▼
ES8311 codec via I²S  (LVDS / LCD mode)
  — or —
LT8912B embedded I²S audio  (HDMI mode)
```

The audio task naturally paces itself by blocking on the codec write, which takes approximately 20 ms per buffer (882 stereo 16-bit samples at 44100 Hz / 50 fps). No explicit timing is needed in the callback.

### Registering a callback

```c
void my_audio_cb(void *userdata, int16_t *stream, int length)
{
    // fill stream with 'length' bytes of signed 16-bit stereo PCM
}

audio_set_callback(my_audio_cb, NULL);
```

The buffer size is fixed at `AY_FREQUENCY * AY_CHANNEL * (AY_BITS/8) / 50` = **3528 bytes** (882 stereo samples).

### Audio constants (defines.h)

```c
#define AY_FREQUENCY  44100   // Hz
#define AY_CHANNEL    2       // stereo
#define AY_BITS       16
```

---

## Input

The input state is always available globally, updated once per `mini_end_frame()`:

```c
extern volatile input_state_t g_inputState;
```

```c
typedef struct {
    int8_t  j0_x;        // Joystick port 0 X axis: -128 … +127
    int8_t  j0_y;        // Joystick port 0 Y axis: -128 … +127
    int8_t  j1_x;        // Joystick port 1 X axis: -128 … +127
    int8_t  j1_y;        // Joystick port 1 Y axis: -128 … +127
    uint8_t buttonState; // Packed button bits — see below
} input_state_t;
```

### Button bit layout

```
Bit:    7    6    5    4    3    2    1    0
        ──── Port 1 buttons ────  ── Port 0 buttons ──
        B4   B3   B2   B1        B4   B3   B2   B1

Buttons are ZERO-ACTIVE: 0 = pressed, 1 = released.
```

You can also query keys directly:

```c
bool isKeyDown(uint8_t hid_keycode);
bool isAsciiDown(char c);
```

> **Note:** The USB HID stack is currently polled, not interrupt-driven. Use `g_inputState` or the `isKeyDown` helpers; do not spin a separate polling task.

### Physical connector / Bluetooth

GPIO pinout for the 9-pin retro joystick connector and Bluetooth controller support are not yet documented. This section will be updated when that information becomes available.

---

## Events

```c
esp_add_event_listener(my_listener);

void my_listener(int event_id)
{
    switch (event_id)
    {
        case ESP_EVENT_INIT:          // framework (re-)initialised
            break;
        case ESP_EVENT_SIZE_CHANGED:  // display switched between LCD and HDMI
            resize();
            break;
        case ESP_EVENT_KILL:          // another app selected — exit your loop
            tobekilled = 1;
            break;
    }
}
```

| Constant | Value | When fired |
|---|---|---|
| `ESP_EVENT_INIT` | 1 | Framework init or application switch |
| `ESP_EVENT_SIZE_CHANGED` | 2 | LCD ↔ HDMI toggle (`Y` key) |
| `ESP_EVENT_KILL` | 3 | Application should stop and return |
| User-defined | ≥ 100 | Available for application use |

`readevents()` is called automatically inside `mini_end_frame()`, so events are dispatched once per frame without any extra polling code.

---

## Writing Your Own Application

Everything the application needs comes from a **single include**:

```c
#include "defines.h"
```

### Minimal skeleton

```c
#include "defines.h"

static volatile int tobekilled   = 0;
static volatile int overlayEnabled = 1;
static int mode; // VIDEO_OUT_HDMI or VIDEO_OUT_LVDS

static void resize(void)
{
    // getDisplayWidth() / getDisplayHeight() return the current logical size
}

static void my_event_listener(int event_id)
{
    if (event_id == ESP_EVENT_KILL)          tobekilled = 1;
    if (event_id == ESP_EVENT_SIZE_CHANGED)  resize();
}

static void my_audio_cb(void *ud, int16_t *stream, int length)
{
    // generate 'length' bytes of stereo 16-bit PCM
}

IRAM_ATTR void my_app(void)
{
    // 1. Load assets
    int w = (mode == VIDEO_OUT_HDMI) ? HDMI_OVERLAY_WIDTH  : LCD_OVERLAY_WIDTH;
    int h = (mode == VIDEO_OUT_HDMI) ? HDMI_OVERLAY_HEIGHT : LCD_OVERLAY_HEIGHT;
    loadOverlayRGB("/sdcard/my_overlay.png", w, h);

    // 2. Register with the framework
    yuv_palette_init();
    setAppFPS(50);
    esp_add_event_listener(my_event_listener);
    audio_set_callback(my_audio_cb, NULL);

    // 3. Main loop
    tobekilled = 0;
    while (!tobekilled)
    {
        // update simulation state …

        mini_draw_line(x0, y0, x1, y1, brightness);
        mini_draw_line_color(x0, y0, x1, y1, YUV_PALETTE_GREEN, brightness);

        mini_end_frame();   // submit frame + refresh input
    }

    // 4. Clean up and return — framework will call my_app() again
}
```

### Coordinate system

| Mode | Logical width | Logical height |
|---|---|---|
| LVDS / LCD (portrait) | `getDisplayWidth()` → 480 | `getDisplayHeight()` → 800 |
| HDMI | `getDisplayWidth()` → 1280 | `getDisplayHeight()` → 720 |

`getScreenWidth()` / `getScreenHeight()` are convenience aliases for the same values. The 90° rotation for the LCD is applied transparently inside `mini_draw_line*` — the application always works in logical coordinates.

---

## API Reference

### Drawing

```c
// Greyscale line (overlay-aware when an overlay is loaded)
void mini_draw_line(int x0, int y0, int x1, int y1, uint8_t brightness);

// Palette-coloured line (color = index 0-127 from YUV_PALETTE_* constants)
void mini_draw_line_color(int x0, int y0, int x1, int y1, int color, uint8_t brightness);

// Submit the current frame to the renderer (also refreshes input state)
void mini_end_frame(void);
```

### Display size

```c
int getDisplayWidth(void);   // logical screen width
int getDisplayHeight(void);  // logical screen height
int getScreenWidth(void);    // alias for getDisplayWidth
int getScreenHeight(void);   // alias for getDisplayHeight
```

### Frame rate

```c
setAppFPS(int fps);
// sets g_fpsToReach = 1000000 / fps (microseconds per frame)
// mini_end_frame does NOT throttle automatically — the application must
// measure elapsed time and delay if needed (see vectrex example above)
```

### Palette

```c
void yuv_palette_init(void);
// Initialises s_overlay_palette_yuv[128][3] with BT.601 YUV values.
// Call once before using mini_draw_line_color().
```

### Overlay

```c
void loadOverlayRGB(const char *sdcard_path, int display_width, int display_height);
// Decodes PNG, builds palette, blits to both hardware frame buffers.
// Pass HDMI_OVERLAY_WIDTH/HEIGHT or LCD_OVERLAY_WIDTH/HEIGHT as appropriate.
```

### Line visual parameters

```c
extern int g_color_mode;      // 0 = greyscale, 1 = colour
extern int g_line_width;      // stroke width in pixels
extern int g_line_glow;       // glow halo width
extern int brightnessAdjust;  // base brightness (>255 causes pixel spill)

void changeGlobalLineValues(void);
// MUST be called after changing any of the above — recalculates
// g_line_Rb, g_beam_r, g_beam_r2_q16, g_gs used by the assembly routines.
```

### Audio

```c
typedef void (*audio_sample_callback_t)(void *userdata, int16_t *stream, int length);
void audio_set_callback(audio_sample_callback_t cb, void *userdata);
```

### Input

```c
extern volatile input_state_t g_inputState; // always current after mini_end_frame()
bool isKeyDown(uint8_t hid_keycode);
bool isAsciiDown(char c);
```

### Events

```c
void esp_add_event_listener(void (*listener)(int event_id));
// ESP_EVENT_INIT = 1, ESP_EVENT_SIZE_CHANGED = 2, ESP_EVENT_KILL = 3
```

---

## Global Tuning Parameters

These constants in `defines.h` control compile-time defaults. Runtime changes to the line parameters require a call to `changeGlobalLineValues()`.

| Constant | Default | Description |
|---|---|---|
| `MAX_EMU_FPS` | 50 | Default frame rate target |
| `LINE_WIDTH` | 1 | Default line stroke width (pixels) |
| `LINE_GLOW_WIDTH` | 2 | Default glow halo width (pixels) |
| `BRIGHTNESS_ADJUST` | 50 | Default brightness level |
| `GLOBAL_OVERLAY_ALPHA` | 50 | Semi-transparent overlay pixel alpha (0–255) |
| `ENABLE_OVERLAYS` | 0 | Compile-time overlay feature toggle |
| `AY_FREQUENCY` | 44100 | Audio sample rate (Hz) |
| `AY_CHANNEL` | 2 | Audio channels (stereo) |
| `AY_BITS` | 16 | Audio bit depth |

---

## Key Bindings

All key bindings are compatible with the **VIDE** Vectrex emulator keyboard layout.

### Joystick / Buttons

| Key | Maps to |
|---|---|
| Arrow keys | Joystick port 0 (X / Y axis) |
| `A` `S` `D` `F` | Port 0 buttons 1 – 4 |
| `H` `J` `U` `N` | Joystick port 1 (X / Y axis) |
| `Q` `W` `E` `R` | Port 1 buttons 1 – 4 |

### Global System Keys

| Key | Function |
|---|---|
| `F1` / `F2` | Decrease / increase line width |
| `F3` / `F4` | Decrease / increase line glow |
| `F11` / `F12` | Decrease / increase software brightness |
| `B` / `V` | Decrease / increase hardware brightness *(LCD only)* |
| `M` | Switch to next application (if multiple registered) |
| `Y` | Toggle between HDMI and LCD output |
| `SPACE` | Toggle overlay on / off |

---

## Directory Structure

```
ESP32_P4_Vectrex/
├── CMakeLists.txt              project root; -O3, project name "vectrex"
├── partitions.csv              custom partition table
├── sdkconfig.defaults          IDF config defaults
│
├── components/
│   └── sd_card/                SD card FAT component
│
└── main/
    ├── defines.h               ← single include for applications
    ├── board.h                 GPIO assignments, bus widths, pixel format
    ├── main.c                  task setup, renderer, frame pipeline,
    │                           line API (mini_draw_line*), input,
    │                           event dispatch, yuv_palette_init
    ├── audio.i                 ES8311 / I²S audio init and task
    ├── overlay.i               PNG load, palettise, blit
    ├── usb.i                   USB HID keyboard polling
    ├── sdcard.i                SD mount / unmount helpers
    ├── file.i                  ROM file loader
    ├── esp_events.h/.i         event IDs and dispatch helpers
    │
    ├── draw_line_yuv422.h              line-drawing API declarations
    ├── draw_line_yuv422_brightness.c   greyscale line — C reference
    ├── draw_line_yuv422_brightness.S   greyscale line — RISC-V ASM (used)
    ├── draw_line_yuv422_color.c        palette-colour line — C reference
    ├── draw_line_yuv422_color.S        palette-colour line — RISC-V ASM (used)
    ├── draw_line_yuv422_overlay.c      overlay-aware line — C reference
    ├── draw_line_yuv422_overlay.S      overlay-aware line — RISC-V ASM (used)
    ├── draw_line_yuv422_color_rgbPalette.c   palette helpers
    │
    ├── hdmi.c / hdmi.h         LT8912B HDMI 1280×720 @ 60 Hz init
    ├── lvds.c / lvds.h         LT8912B LVDS / LCD 480×800 init
    ├── lodepng.c / lodepng.h   PNG decoder
    │
    ├── vecx/                   Vectrex emulator core
    │   ├── vecx.c / vecx.h     main loop, VIA, DAC, integrators
    │   ├── e6809.h / .i        Motorola 6809 CPU core
    │   ├── e8910.c / e8910.h   AY-3-8910 sound chip
    │   └── libayemu/           AY-3-8912 sound library
    │
    └── vsim/                   Atari vector-arcade simulator (Tempest, etc.)
        ├── display.c / display.h   AVG / DVG state machine
        ├── game.c / game.h         game selection and ROM loading
        ├── memory.c / memory.h     6502 address space, VRAM, colour RAM
        ├── arcadia.c               Tempest / Quantum hardware config
        ├── vx_interface.c          draw_line2 bridge to the line framework
        ├── framework.c             emulator main loop
        ├── sim6502.h               6502 CPU core
        ├── macro6502.h             6502 addressing-mode macros
        ├── mathbox.c / mathbox.h   Tempest MATHBOX chip simulation
        ├── pokey.c / pokey.h       Atari POKEY sound chip
        └── dis6502.c / dis6502.h   6502 disassembler (debug)
```

---

## Build

### Prerequisites

1. Install **ESP-IDF 5.5.1** — ESP-IDF 6.x is **not** compatible with this ESP32-P4 revision.
2. Install **Visual Studio Code** with the **ESP-IDF extension** (recommended workflow).
3. Place ROM files and overlay PNGs on an SD card formatted as FAT.

### Configure

```bash
idf.py set-target esp32p4
idf.py menuconfig          # optional; sensible defaults are in sdkconfig.defaults
```

### Build and flash

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### Managed components

The LT8912B LCD/HDMI bridge driver (`esp_lcd_lt8912b`) is pulled in as an IDF managed component. It is declared in `main/idf_component.yml` and pinned in `dependencies.lock` — no manual installation required.

---

## Emulators Included

### Vectrex

A derivative of **VecX** with enhancements from VIDE, PiTrex, and other community Vectrex projects. Not every hardware quirk is emulated — some accuracy was traded for performance on the ESP32-P4.

- **CPU:** Motorola 6809 (`e6809`)
- **Sound:** AY-3-8910 via `libayemu`
- **Vector output:** integrator simulation → `mini_draw_line()`

### Atari AVG / DVG — vsim

Simulation of the Atari **Analog Vector Generator** (AVG) and **Digital Vector Generator** (DVG) state machines, covering Tempest and related titles.

The AVG is a 256 × 4-bit PROM-driven state machine clocked at 1.5 MHz. The simulation executes each PROM state transition in software and feeds the resulting vectors to the line-drawing framework via `draw_line2()` in `vx_interface.c`.

---

*Contributions, bug reports, and hardware notes (especially GPIO pinout for the 9-pin retro connector) welcome via [GitHub Issues](https://github.com/malbanGit/ESP32_P4_Vectrex/issues).*
