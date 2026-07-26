
/* Write s_overlay (BGRA, 4 bytes/pixel) into s_fb_back (BGR, 3 bytes/pixel).
 * Destination is assumed to be black, so partial-alpha pixels scale the source
 * colour directly: out = src * effective_alpha / 255.
 *   alpha == 0   → skip (dest stays black)
 *   alpha == 255 → straight copy
 *   otherwise    → alpha clamped by alphaAdjust, then out = src * a / 255     */
void drawOverlay(uint8_t *dest)
{
    if (s_overlay == NULL) return;

    const uint8_t *src  = s_overlay;
    int total = LCD_H_RES * LCD_V_RES;

    for (int i = 0; i < total; i++, src += 4, dest += 3)
    {
        uint8_t a = src[3];
        if (a == 0) /* fully transparent — leave black */
        {
            dest[0] = 0;
            dest[1] = 0;
            dest[2] = 0;
        }
        else if (a == 255) {                  /* fully opaque — straight copy */
            dest[0] = src[0];
            dest[1] = src[1];
            dest[2] = src[2];
        } else {                         /* semi-transparent: scale over black */
            int ea = (int)s_overlay_alpha_val;
            if (ea <= 0)  continue;
            if (ea > 255) ea = 255;
            dest[0] = (uint8_t)((src[0] * ea) >> 8);
            dest[1] = (uint8_t)((src[1] * ea) >> 8);
            dest[2] = (uint8_t)((src[2] * ea) >> 8);
        }
    }
}
/* Write s_overlay_pal (palette, 1 byte/pixel) into dest.
 *
 * Palette index byte:
 *   bit 7 clear → fully opaque pixel, use palette colour directly
 *   bit 7 set   → semi-transparent, scale by s_overlay_alpha_val
 *
 * Two pixel-format paths selected at compile time via VIDEO_FB_YUV422:
 *
 *   RGB888 (VIDEO_FB_YUV422 == 0):
 *     dest is 3 bytes/pixel (B, G, R order).
 *     Palette stores BGR so values are written straight through.
 *
 *   YUV422 / YUYV (VIDEO_FB_YUV422 == 1):
 *     dest is 2 bytes/pixel — memory layout per pixel pair:
 *       [Y0][U][Y1][V]  (YUYV, little-endian)
 *     Y  is computed per pixel  (BT.601 full-range: Y = (77R+150G+29B)>>8)
 *     U/V are written once per even-column pixel and shared with the
 *     following odd-column pixel.  An overlay starting at an odd column
 *     gets correct luma on the first pixel but inherits the initialised
 *     U=V=128 (neutral gray) chroma for that one pixel — acceptable.
 */
void drawOverlayPal(uint8_t *dest)
{
    if (s_overlay_pal == NULL) return;

    for (int y = 0; y < s_ov_h; y++)
    {
        const uint8_t *row_pal = s_overlay_pal + (size_t)y * s_ov_w;
        int dst_y = s_ov_off_y + y;

#if VIDEO_FB_YUV422
        uint8_t *row_dst = dest + (size_t)dst_y * LCD_H_RES * 2;

        for (int x = 0; x < s_ov_w; x++)
        {
            int px = s_ov_off_x + x;
            uint8_t pidx = row_pal[x];
            const uint8_t *c = s_overlay_palette[pidx & 0x7F];
            int b, g, r;
            if (!(pidx & 0x80)) {
                b = c[0]; g = c[1]; r = c[2];
            } else {
                b = (c[0] * s_overlay_alpha_val) >> 8;
                g = (c[1] * s_overlay_alpha_val) >> 8;
                r = (c[2] * s_overlay_alpha_val) >> 8;
            }

            /* Y (BT.601 full-range: 77+150+29 = 256) */
            int yv = (77 * r + 150 * g + 29 * b) >> 8;
            if (yv > 255) yv = 255;
            row_dst[px * 2] = (uint8_t)yv;

            /* U and V written only at even columns; covers this pixel pair */
            if (!(px & 1)) {
                int uv = 128 + ((-43 * r -  85 * g + 128 * b) >> 8);
                int vv = 128 + ((128 * r - 107 * g -  21 * b) >> 8);
                if (uv < 0) uv = 0; else if (uv > 255) uv = 255;
                if (vv < 0) vv = 0; else if (vv > 255) vv = 255;
                row_dst[px * 2 + 1] = (uint8_t)uv;
                row_dst[px * 2 + 3] = (uint8_t)vv;
            }
        }
#else // else is RGB
        uint8_t *row_dst = dest + ((size_t)dst_y * LCD_H_RES + s_ov_off_x) * 3;

        for (int x = 0; x < s_ov_w; x++, row_dst += 3)
        {
            uint8_t pidx = row_pal[x];
            const uint8_t *c = s_overlay_palette[pidx & 0x7F];
            if (!(pidx & 0x80)) {
                row_dst[0] = c[0];
                row_dst[1] = c[1];
                row_dst[2] = c[2];
            } else {
                row_dst[0] = (uint8_t)((c[0] * s_overlay_alpha_val) >> 8);
                row_dst[1] = (uint8_t)((c[1] * s_overlay_alpha_val) >> 8);
                row_dst[2] = (uint8_t)((c[2] * s_overlay_alpha_val) >> 8);
            }
        }
#endif
    }
}

// ---------------------------------------------------------------------------
// overlay_load_png_bgra  — decode PNG into a BGRA (4 bytes/pixel) buffer.
// lodepng gives RGBA; we store as BGRA to match the framebuffer's BGR order.
// fb must be fb_w * fb_h * 4 bytes.
// ---------------------------------------------------------------------------
esp_err_t overlay_load_png_bgra(const char *path, uint8_t *fb, int fb_w, int fb_h)
{
    unsigned char *raw   = NULL;
    unsigned       width = 0, height = 0;

    unsigned err = lodepng_decode32_file(&raw, &width, &height, path);
    if (err) {
        ESP_LOGE(TAG, "PNG decode failed (%u): %s", err, lodepng_error_text(err));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "PNG %s  %u×%u → %d×%d (BGRA)", path, width, height, fb_w, fb_h);

    for (int y = 0; y < fb_h; y++) {
        unsigned src_y = (unsigned)((uint64_t)y * height / (unsigned)fb_h);
        for (int x = 0; x < fb_w; x++) {
            unsigned src_x = (unsigned)((uint64_t)x * width / (unsigned)fb_w);
            const unsigned char *px = raw + (src_y * width + src_x) * 4;
            uint8_t *dst = fb + ((size_t)y * fb_w + x) * 4;
            dst[0] = px[2]; // B  (swap R↔B for BGR framebuffer)
            dst[1] = px[1]; // G
            dst[2] = px[0]; // R
            dst[3] = px[3]; // A  (preserved)
        }
    }

    free(raw);
    return ESP_OK;
}

/* ── Palette builder: frequency-ranked 4-bit histogram ──────────────────────
 * idx = (R>>4)<<8 | (G>>4)<<4 | (B>>4)  →  4096 buckets, 16 KB DRAM.
 * Fills palette with the ≤127 most-frequent colour buckets, most common first.
 * Bucket centre = (nibble << 4) | 8  (midpoint of the 16-unit range).
 * Transparent (alpha==0) pixels are excluded.
 */
static int build_palette_freq(
        const uint8_t *overlay_bgra,
        int off_x, int off_y,
        int img_w, int img_h,
        int lcd_w,
        uint8_t palette[128][3])
{
    static uint32_t hist[4096];   /* 4-bit per channel: 16×16×16 buckets, DRAM */
    memset(hist, 0, sizeof(hist));

    for (int y = 0; y < img_h; y++) {
        const uint8_t *row = overlay_bgra + ((size_t)(off_y + y) * lcd_w + off_x) * 4;
        for (int x = 0; x < img_w; x++, row += 4) {
            if (row[3] == 0) continue;
            uint32_t idx = ((uint32_t)(row[2] >> 4) << 8)   /* R → bits[11:8] */
                         | ((uint32_t)(row[1] >> 4) << 4)   /* G → bits[7:4]  */
                         |  (uint32_t)(row[0] >> 4);        /* B → bits[3:0]  */
            hist[idx]++;
        }
    }

    /* Pick top-127 buckets by pixel count (partial selection, O(127×4096)). */
    int n = 0;
    while (n < 127) {
        uint32_t best_cnt = 0;
        int      best_idx = -1;
        for (int i = 0; i < 4096; i++) {
            if (hist[i] > best_cnt) { best_cnt = hist[i]; best_idx = i; }
        }
        if (best_idx < 0) break;
        hist[best_idx] = 0;   /* remove from future rounds */

        palette[n][0] = (uint8_t)(((best_idx      ) & 0xF) << 4 | 8);  /* B */
        palette[n][1] = (uint8_t)(((best_idx >>  4) & 0xF) << 4 | 8);  /* G */
        palette[n][2] = (uint8_t)(((best_idx >>  8) & 0xF) << 4 | 8);  /* R */
        n++;
    }
    return n;
}

// returns pointer
/* Load a PNG (with alpha), scale it to (img_w x img_h), and centre it on a
 * full-screen BGRA overlay buffer (LCD_H_RES x LCD_V_RES, 4 bytes/pixel).
 * Surrounding area is filled with transparent black (alpha=0).
 * Pass img_w=0 / img_h=0 to stretch to full screen.                      */
char lastOverlay[MAX_OVERLAY_NAME];
esp_err_t loadOverlayRGB(char *name, int img_w, int img_h)
{
    strncpy(lastOverlay, name, MAX_OVERLAY_NAME-1);

    /* clamp / default to full screen */
    if (img_w <= 0 || img_w > LCD_H_RES) img_w = LCD_H_RES;
    if (img_h <= 0 || img_h > LCD_V_RES) img_h = LCD_V_RES;

    /* free previous overlay buffers */
    if (s_overlay != NULL)    { heap_caps_free(s_overlay);    s_overlay    = NULL; }
    if (s_overlay_bg != NULL) { heap_caps_free(s_overlay_bg); s_overlay_bg = NULL; }
    if (s_overlay_pal != NULL){ heap_caps_free(s_overlay_pal);s_overlay_pal= NULL; }

    void clearFramebuffers();
    clearFramebuffers();
    if (!overlayEnabled) return ESP_OK;

    s_overlay_pal_n = 0;
    s_ov_w = 0;

    /* allocate full-screen BGRA overlay buffer in PSRAM (4 bytes/pixel) */
    size_t buf_sz = (size_t)LCD_H_RES * LCD_V_RES * 4;
    s_overlay = heap_caps_malloc(buf_sz, MALLOC_CAP_SPIRAM);
    if (!s_overlay)
    {
        ESP_LOGE(TAG, "PSRAM alloc failed (%u bytes)", buf_sz);
        return ESP_ERR_NO_MEM;
    }

    /* fill entire buffer with transparent black */
    memset(s_overlay, 0, buf_sz);
    uint8_t *scaled=NULL;
    /* decode + scale PNG into a temporary BGRA buffer */
    if (mode ==VIDEO_OUT_HDMI)
    {
        scaled = heap_caps_malloc((size_t)img_w * img_h * 4, MALLOC_CAP_SPIRAM);
        if (!scaled)
        {
            ESP_LOGE(TAG, "PSRAM alloc for scaled image failed");
            heap_caps_free(s_overlay);
            s_overlay = NULL;
            return ESP_ERR_NO_MEM;
        }

        esp_err_t ret = overlay_load_png_bgra(name, scaled, img_w, img_h);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "PNG load failed");
            heap_caps_free(scaled);
            heap_caps_free(s_overlay);
            s_overlay = NULL;
            return ret;
        }
    }
    else
    {
        /* LCD portrait: load PNG in landscape orientation (swapped dims), then
        * rotate 90° CW so it fills the portrait screen.                       */
        int load_w = img_h, load_h = img_w;
        scaled = heap_caps_malloc((size_t)load_w * load_h * 4, MALLOC_CAP_SPIRAM);
        if (!scaled)
        {
            ESP_LOGE(TAG, "PSRAM alloc for scaled image failed");
            heap_caps_free(s_overlay);
            s_overlay = NULL;
            return ESP_ERR_NO_MEM;
        }

        esp_err_t ret = overlay_load_png_bgra(name, scaled, load_w, load_h);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "PNG load failed");
            heap_caps_free(scaled);
            heap_caps_free(s_overlay);
            s_overlay = NULL;
            return ret;
        }

        /* rotate 90° CW: src pixel (sx=dy, sy=load_h-1-dx) → dst pixel (dx, dy) */
        uint8_t *rotated = heap_caps_malloc((size_t)img_w * img_h * 4, MALLOC_CAP_SPIRAM);
        if (!rotated)
        {
            ESP_LOGE(TAG, "PSRAM alloc for rotated image failed");
            heap_caps_free(scaled);
            heap_caps_free(s_overlay);
            s_overlay = NULL;
            return ESP_ERR_NO_MEM;
        }
        for (int dy = 0; dy < img_h; dy++) {
            for (int dx = 0; dx < img_w; dx++) {
                const uint8_t *s = scaled  + ((size_t)(load_h - 1 - dx) * load_w + dy) * 4;
                uint8_t       *d = rotated + ((size_t)dy * img_w + dx) * 4;
                d[0] = s[0]; d[1] = s[1]; d[2] = s[2]; d[3] = s[3];
            }
        }
        heap_caps_free(scaled);
        scaled = rotated;
    }

    /* centre position */
    int off_x = (LCD_H_RES - img_w) / 2;
    int off_y = (LCD_V_RES - img_h) / 2;

    /* blit scaled image into the centre of the full-screen buffer */
    for (int y = 0; y < img_h; y++)
    {
        const uint8_t *src = scaled    + (size_t)y * img_w * 4;
        uint8_t       *dst = s_overlay + ((size_t)(off_y + y) * LCD_H_RES + off_x) * 4;
        memcpy(dst, src, (size_t)img_w * 4);
    }

    heap_caps_free(scaled);

    /* Build precomputed RGB888 background (overlay-over-black).
     * undraw_line_rgb888_overlay restores from this — no per-pixel alpha
     * math in the hot path, just two memcpys per bounding-box row.        */
    size_t bg_sz = (size_t)LCD_H_RES * LCD_V_RES * 3;
    s_overlay_bg = heap_caps_malloc(bg_sz, MALLOC_CAP_SPIRAM);
    if (s_overlay_bg) 
    {
        const uint8_t *src = s_overlay;
        uint8_t       *dst = s_overlay_bg;
        int total = LCD_H_RES * LCD_V_RES;
        for (int i = 0; i < total; i++, src += 4, dst += 3) {
            uint8_t a = src[3];
            if (a == 0) 
            {
                dst[0] = dst[1] = dst[2] = 0;
            } 
            else if (a == 255) 
            {
                dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2];
            } 
            else 
            {
                dst[0] = (uint8_t)((src[0] * s_overlay_alpha_val ) >> 8);
                dst[1] = (uint8_t)((src[1] * s_overlay_alpha_val ) >> 8);
                dst[2] = (uint8_t)((src[2] * s_overlay_alpha_val ) >> 8);
            }
        }
    }

    /* ── Build palettised index for the active region ──────────────────── */
    s_ov_off_x = off_x;
    s_ov_off_y = off_y;
    s_ov_w     = img_w;
    s_ov_h     = img_h;

    s_overlay_pal = heap_caps_malloc((size_t)img_w * img_h, MALLOC_CAP_SPIRAM);
    if (s_overlay_pal) 
    {
        s_overlay_pal_n = 0;
        memset(s_overlay_palette, 0, sizeof(s_overlay_palette));

        /* Pass 1: median-cut quantisation → 127-entry BGR palette. */
        s_overlay_pal_n = build_palette_freq(
                s_overlay, off_x, off_y, img_w, img_h, LCD_H_RES,
                s_overlay_palette);

        /* Pass 2: assign index bytes. */
        for (int y = 0; y < img_h; y++) 
        {
            const uint8_t *src = s_overlay + ((size_t)(off_y + y) * LCD_H_RES + off_x) * 4;
            uint8_t       *dst = s_overlay_pal + (size_t)y * img_w;
            for (int x = 0; x < img_w; x++, src += 4, dst++) 
            {
                uint8_t b = src[0], g = src[1], r = src[2], a = src[3];
                int best = 0, best_d = 0x7FFFFFFF;
                
                for (int i = 0; i < s_overlay_pal_n; i++) 
                {
                    int db = (int)b - s_overlay_palette[i][0];
                    int dg = (int)g - s_overlay_palette[i][1];
                    int dr = (int)r - s_overlay_palette[i][2];
                    int d  = db*db + dg*dg + dr*dr;
                    if (d < best_d) { best = i; best_d = d; }
                    if (d == 0) break;
                }
                if (a <= 0 || a >= 255) 
                { 
                    *dst = (uint8_t)(best & (~0x80));
                }
                else
                {
                    *dst = (uint8_t)(0x80 | best);
                }
            }
        }
        /* Build YUV palette caches used by the YUV422 distance-field renderer. */
        {
            int ea = s_overlay_alpha_val;
            for (int i = 0; i < 128; i++) {
                int b = s_overlay_palette[i][0];
                int g = s_overlay_palette[i][1];
                int r = s_overlay_palette[i][2];
                /* full-range YUV for draw blending */
                s_overlay_palette_yuv[i][0] = (uint8_t)((77*r + 150*g + 29*b) >> 8);
                int u = 128 + ((-43*r - 85*g + 128*b) >> 8);
                int v = 128 + ((128*r - 107*g - 21*b) >> 8);
                s_overlay_palette_yuv[i][1] = (uint8_t)(u < 0 ? 0 : u > 255 ? 255 : u);
                s_overlay_palette_yuv[i][2] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
                /* ea-scaled YUV for undraw restore */
                int bs = (b * ea) >> 8, gs = (g * ea) >> 8, rs = (r * ea) >> 8;
                s_overlay_palette_yuv_ea[i][0] = (uint8_t)((77*rs + 150*gs + 29*bs) >> 8);
                int ue = 128 + ((-43*rs - 85*gs + 128*bs) >> 8);
                int ve = 128 + ((128*rs - 107*gs - 21*bs) >> 8);
                s_overlay_palette_yuv_ea[i][1] = (uint8_t)(ue < 0 ? 0 : ue > 255 ? 255 : ue);
                s_overlay_palette_yuv_ea[i][2] = (uint8_t)(ve < 0 ? 0 : ve > 255 ? 255 : ve);
            }
        }
        ESP_LOGI(TAG, "overlay pal: %d colours, alpha_val=%d", s_overlay_pal_n, s_overlay_alpha_val);
    }
    drawOverlayPal(s_fb_front);
    drawOverlayPal(s_fb_back);

    ESP_LOGI(TAG, "overlay: %s scaled to %dx%d, centred on %dx%d screen (BGRA)",
             name, img_w, img_h, LCD_H_RES, LCD_V_RES);
    return ESP_OK;
}