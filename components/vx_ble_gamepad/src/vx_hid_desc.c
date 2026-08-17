/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * HID report descriptor parser — see vx_hid_desc.h for the rationale.
 */

#include <string.h>

#include "esp_log.h"

#include "vx_hid_desc.h"

static const char *TAG = "VX_HID_DESC";

/* ---- item encoding (HID 1.11 section 6.2.2.2) ----------------------------- */

#define ITEM_TYPE_MAIN   0
#define ITEM_TYPE_GLOBAL 1
#define ITEM_TYPE_LOCAL  2

/* Main tags */
#define TAG_INPUT          0x8
#define TAG_OUTPUT         0x9
#define TAG_COLLECTION     0xA
#define TAG_FEATURE        0xB
#define TAG_END_COLLECTION 0xC

/* Global tags */
#define TAG_USAGE_PAGE    0x0
#define TAG_LOGICAL_MIN   0x1
#define TAG_LOGICAL_MAX   0x2
#define TAG_PHYSICAL_MIN  0x3
#define TAG_PHYSICAL_MAX  0x4
#define TAG_UNIT_EXPONENT 0x5
#define TAG_UNIT          0x6
#define TAG_REPORT_SIZE   0x7
#define TAG_REPORT_ID     0x8
#define TAG_REPORT_COUNT  0x9
#define TAG_PUSH          0xA
#define TAG_POP           0xB

/* Local tags */
#define TAG_USAGE     0x0
#define TAG_USAGE_MIN 0x1
#define TAG_USAGE_MAX 0x2

#define LONG_ITEM_PREFIX 0xFE

#define MAX_LOCAL_USAGES 32
#define GLOBAL_STACK_DEPTH 4

typedef struct {
    uint16_t usage_page;
    int32_t  logical_min;
    int32_t  logical_max;
    uint8_t  report_size;
    uint8_t  report_count;
    uint8_t  report_id;
} global_state_t;

typedef struct {
    /* Running bit offset per (report_id, type).  Reports are looked up in
     * desc->reports, so the offset lives there; this struct only carries the
     * parse-time scratch state. */
    global_state_t glob;
    global_state_t stack[GLOBAL_STACK_DEPTH];
    uint8_t        stack_depth;

    uint32_t usages[MAX_LOCAL_USAGES];
    uint8_t  usage_count;
    uint32_t usage_min;
    uint32_t usage_max;
    bool     usage_range_set;
    bool     local_overflow;
} parse_state_t;

/* ---- little helpers ------------------------------------------------------- */

static uint32_t item_data_unsigned(const uint8_t *p, uint8_t size)
{
    uint32_t v = 0;
    for (uint8_t i = 0; i < size; ++i) {
        v |= (uint32_t)p[i] << (8 * i);
    }
    return v;
}

/* Logical Minimum/Maximum are signed (HID 1.11 section 6.2.2.7).  This is why
 * descriptors write 255 as "0x26 0xFF 0x00" (two bytes) and not "0x25 0xFF",
 * which would mean -1. */
static int32_t item_data_signed(const uint8_t *p, uint8_t size)
{
    uint32_t v = item_data_unsigned(p, size);

    if (size > 0 && size < 4) {
        uint32_t sign_bit = 1u << (8 * size - 1);
        if (v & sign_bit) {
            v |= ~((sign_bit << 1) - 1);
        }
    }
    return (int32_t)v;
}

static void locals_clear(parse_state_t *st)
{
    st->usage_count     = 0;
    st->usage_min       = 0;
    st->usage_max       = 0;
    st->usage_range_set = false;
    /* local_overflow is deliberately NOT cleared: it is a whole-parse flag. */
}

/* Which usage goes to the i-th control of a Main item.
 *
 * HID 1.11 section 6.2.2.8: if there are fewer usages than controls, the last
 * usage is repeated for the remaining ones.  A Usage Minimum/Maximum pair
 * instead assigns consecutive usages, which is how button banks are declared
 * ("Usage Minimum (Button 1) / Usage Maximum (Button 16)"). */
static uint32_t usage_for_index(const parse_state_t *st, uint8_t index)
{
    if (st->usage_range_set) {
        uint32_t u = st->usage_min + index;
        return (u <= st->usage_max) ? u : 0;
    }
    if (st->usage_count == 0) {
        return 0;
    }
    return (index < st->usage_count) ? st->usages[index]
                                     : st->usages[st->usage_count - 1];
}

/* ---- report bookkeeping --------------------------------------------------- */

static vx_hid_report_t *report_get_or_add(vx_hid_desc_t *desc, uint8_t report_id,
                                          uint8_t type)
{
    for (uint8_t i = 0; i < desc->report_count; ++i) {
        if (desc->reports[i].report_id == report_id &&
            desc->reports[i].type == type) {
            return &desc->reports[i];
        }
    }
    if (desc->report_count >= VX_HID_MAX_REPORTS) {
        desc->truncated = true;
        return NULL;
    }

    vx_hid_report_t *r = &desc->reports[desc->report_count++];
    r->report_id   = report_id;
    r->type        = type;
    r->bit_len     = 0;
    r->field_first = desc->field_count;
    r->field_count = 0;
    return r;
}

/* ---- Main item: this is where fields are emitted -------------------------- */

static void emit_main(vx_hid_desc_t *desc, parse_state_t *st, uint8_t type,
                      uint32_t data)
{
    vx_hid_report_t *rep = report_get_or_add(desc, st->glob.report_id, type);
    uint8_t flags = (uint8_t)(data & 0x07); /* Constant / Variable / Relative */

    if (!rep) {
        return;
    }

    /* Fields of a given report must be contiguous in the pool, because a report
     * stores {first, count}.  Interleaved reports (Input for ID 1, then Input
     * for ID 2, then Input for ID 1 again) would break that.  Real descriptors
     * do not do this, but detect it rather than corrupt the mapping. */
    if (rep->field_count != 0 &&
        rep->field_first + rep->field_count != desc->field_count) {
        ESP_LOGW(TAG, "interleaved report id %u type %u: fields dropped",
                 rep->report_id, type);
        desc->truncated = true;
        return;
    }

    for (uint8_t i = 0; i < st->glob.report_count; ++i) {
        if (desc->field_count >= VX_HID_MAX_FIELDS) {
            desc->truncated = true;
            return;
        }

        vx_hid_field_t *f = &desc->fields[desc->field_count];

        f->usage_page  = st->glob.usage_page;
        f->bit_offset  = rep->bit_len;
        f->bit_size    = st->glob.report_size;
        f->flags       = flags;
        f->logical_min = st->glob.logical_min;
        f->logical_max = st->glob.logical_max;

        if (flags & VX_HID_FLAG_CONSTANT) {
            f->usage = 0; /* padding carries no usage */
        } else {
            uint32_t u = usage_for_index(st, i);
            /* A 4-byte Usage item embeds its own page in the high half-word. */
            if (u > 0xFFFF) {
                f->usage_page = (uint16_t)(u >> 16);
                f->usage      = (uint16_t)(u & 0xFFFF);
            } else {
                f->usage = (uint16_t)u;
            }
        }

        rep->bit_len += st->glob.report_size;
        rep->field_count++;
        desc->field_count++;
    }
}

/* ---- parser --------------------------------------------------------------- */

bool vx_hid_desc_parse(const uint8_t *raw, size_t len, vx_hid_desc_t *out)
{
    parse_state_t st;
    size_t        pos = 0;

    if (!raw || !out || len == 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    memset(&st, 0, sizeof(st));

    while (pos < len) {
        uint8_t prefix = raw[pos++];
        uint8_t bsize, btype, btag;

        if (prefix == LONG_ITEM_PREFIX) {
            /* Long items carry no information we use; skip cleanly rather than
             * desynchronise. */
            if (pos + 2 > len) {
                ESP_LOGE(TAG, "truncated long item at %u", (unsigned)pos);
                return false;
            }
            uint8_t data_size = raw[pos];
            pos += 2 + data_size;
            continue;
        }

        bsize = prefix & 0x03;
        if (bsize == 3) {
            bsize = 4; /* the 2-bit size field encodes 0,1,2,4 */
        }
        btype = (prefix >> 2) & 0x03;
        btag  = (prefix >> 4) & 0x0F;

        if (pos + bsize > len) {
            ESP_LOGE(TAG, "truncated item at %u (need %u bytes, %u left)",
                     (unsigned)pos, bsize, (unsigned)(len - pos));
            /* Partial descriptors are the norm when the host forgets to use a
             * long read, so return what we have instead of nothing. */
            out->truncated = true;
            return out->field_count > 0;
        }

        const uint8_t *data = &raw[pos];
        pos += bsize;

        switch (btype) {
        case ITEM_TYPE_MAIN:
            switch (btag) {
            case TAG_INPUT:
                emit_main(out, &st, VX_HID_REPORT_INPUT,
                          item_data_unsigned(data, bsize));
                locals_clear(&st);
                break;
            case TAG_OUTPUT:
                emit_main(out, &st, VX_HID_REPORT_OUTPUT,
                          item_data_unsigned(data, bsize));
                locals_clear(&st);
                break;
            case TAG_FEATURE:
                emit_main(out, &st, VX_HID_REPORT_FEATURE,
                          item_data_unsigned(data, bsize));
                locals_clear(&st);
                break;
            case TAG_COLLECTION:
            case TAG_END_COLLECTION:
                /* Collections only add structure, not layout.  We ignore the
                 * hierarchy on purpose: gamepads put controls in Application or
                 * Physical collections indifferently, so relying on it would
                 * reject perfectly valid devices. */
                locals_clear(&st);
                break;
            default:
                locals_clear(&st);
                break;
            }
            break;

        case ITEM_TYPE_GLOBAL:
            switch (btag) {
            case TAG_USAGE_PAGE:
                st.glob.usage_page = (uint16_t)item_data_unsigned(data, bsize);
                break;
            case TAG_LOGICAL_MIN:
                st.glob.logical_min = item_data_signed(data, bsize);
                break;
            case TAG_LOGICAL_MAX:
                st.glob.logical_max = item_data_signed(data, bsize);
                break;
            case TAG_REPORT_SIZE:
                st.glob.report_size = (uint8_t)item_data_unsigned(data, bsize);
                break;
            case TAG_REPORT_COUNT:
                st.glob.report_count = (uint8_t)item_data_unsigned(data, bsize);
                break;
            case TAG_REPORT_ID:
                st.glob.report_id   = (uint8_t)item_data_unsigned(data, bsize);
                out->uses_report_ids = true;
                break;
            case TAG_PUSH:
                if (st.stack_depth < GLOBAL_STACK_DEPTH) {
                    st.stack[st.stack_depth++] = st.glob;
                } else {
                    ESP_LOGW(TAG, "global stack overflow, Push ignored");
                }
                break;
            case TAG_POP:
                if (st.stack_depth > 0) {
                    st.glob = st.stack[--st.stack_depth];
                } else {
                    ESP_LOGW(TAG, "global stack underflow, Pop ignored");
                }
                break;
            default:
                /* Physical Min/Max, Unit, Unit Exponent: irrelevant to layout. */
                break;
            }
            break;

        case ITEM_TYPE_LOCAL:
            switch (btag) {
            case TAG_USAGE:
                if (st.usage_count < MAX_LOCAL_USAGES) {
                    st.usages[st.usage_count++] = item_data_unsigned(data, bsize);
                } else {
                    st.local_overflow = true;
                }
                break;
            case TAG_USAGE_MIN:
                st.usage_min       = item_data_unsigned(data, bsize);
                st.usage_range_set = true;
                break;
            case TAG_USAGE_MAX:
                st.usage_max       = item_data_unsigned(data, bsize);
                st.usage_range_set = true;
                break;
            default:
                /* Designator/String indices: not needed. */
                break;
            }
            break;

        default:
            break;
        }
    }

    if (st.local_overflow) {
        ESP_LOGW(TAG, "more than %d usages on one item: some controls unnamed",
                 MAX_LOCAL_USAGES);
        out->truncated = true;
    }

    return out->report_count > 0;
}

const vx_hid_report_t *vx_hid_desc_find_report(const vx_hid_desc_t *desc,
                                               uint8_t report_id,
                                               vx_hid_report_type_t type)
{
    if (!desc) {
        return NULL;
    }
    for (uint8_t i = 0; i < desc->report_count; ++i) {
        if (desc->reports[i].report_id == report_id &&
            desc->reports[i].type == (uint8_t)type) {
            return &desc->reports[i];
        }
    }
    return NULL;
}

bool vx_hid_field_extract(const uint8_t *payload, size_t len,
                          const vx_hid_field_t *field, int32_t *out)
{
    uint32_t value = 0;

    if (!payload || !field || !out || field->bit_size == 0 ||
        field->bit_size > 32) {
        return false;
    }
    if ((size_t)field->bit_offset + field->bit_size > len * 8) {
        return false;
    }

    /* Bit-by-bit rather than a word load: fields routinely straddle byte
     * boundaries (a 4-bit hat after 12 buttons, a 10-bit trigger axis), and the
     * payload has no alignment guarantee. */
    for (uint8_t i = 0; i < field->bit_size; ++i) {
        uint16_t bit = field->bit_offset + i;
        if (payload[bit >> 3] & (1u << (bit & 7))) {
            value |= (1u << i);
        }
    }

    /* Sign-extend only when the control is declared signed.  Getting this wrong
     * turns a centred stick into a value that jumps between 0 and 65535. */
    if (field->logical_min < 0 && field->bit_size < 32) {
        uint32_t sign_bit = 1u << (field->bit_size - 1);
        if (value & sign_bit) {
            value |= ~((sign_bit << 1) - 1);
        }
        *out = (int32_t)value;
    } else {
        *out = (int32_t)value;
    }
    return true;
}

/* ---- diagnostics ---------------------------------------------------------- */

const char *vx_hid_usage_name(uint16_t page, uint16_t usage)
{
    if (page == VX_HID_PAGE_BUTTON) {
        return "Button";
    }
    if (page == VX_HID_PAGE_GENERIC_DESKTOP) {
        switch (usage) {
        case VX_HID_USAGE_X:          return "X";
        case VX_HID_USAGE_Y:          return "Y";
        case VX_HID_USAGE_Z:          return "Z";
        case VX_HID_USAGE_RX:         return "Rx";
        case VX_HID_USAGE_RY:         return "Ry";
        case VX_HID_USAGE_RZ:         return "Rz";
        case VX_HID_USAGE_SLIDER:     return "Slider";
        case VX_HID_USAGE_DIAL:       return "Dial";
        case VX_HID_USAGE_WHEEL:      return "Wheel";
        case VX_HID_USAGE_HAT_SWITCH: return "Hat";
        default:                      return "GenDesk";
        }
    }
    if (page == VX_HID_PAGE_SIMULATION) {
        switch (usage) {
        case VX_HID_USAGE_BRAKE:       return "Brake";
        case VX_HID_USAGE_ACCELERATOR: return "Accel";
        default:                       return "Simu";
        }
    }
    if (page == VX_HID_PAGE_CONSUMER) {
        return "Consumer";
    }
    return "?";
}

static const char *report_type_name(uint8_t t)
{
    switch (t) {
    case VX_HID_REPORT_INPUT:   return "IN ";
    case VX_HID_REPORT_OUTPUT:  return "OUT";
    case VX_HID_REPORT_FEATURE: return "FEA";
    default:                    return "???";
    }
}

void vx_hid_desc_dump(const vx_hid_desc_t *desc, const char *tag)
{
    const char *t = tag ? tag : TAG;

    if (!desc) {
        return;
    }

    ESP_LOGI(t, "parsed descriptor: %u report(s), %u field(s), report IDs %s%s",
             desc->report_count, desc->field_count,
             desc->uses_report_ids ? "used" : "absent",
             desc->truncated ? "  *** TRUNCATED / INCOMPLETE ***" : "");

    for (uint8_t i = 0; i < desc->report_count; ++i) {
        const vx_hid_report_t *r = &desc->reports[i];

        ESP_LOGI(t, "  report id=%u %s  %u bits (%u bytes)  %u fields",
                 r->report_id, report_type_name(r->type), r->bit_len,
                 (r->bit_len + 7) / 8, r->field_count);

        for (uint8_t j = 0; j < r->field_count; ++j) {
            const vx_hid_field_t *f = &desc->fields[r->field_first + j];

            if (f->flags & VX_HID_FLAG_CONSTANT) {
                ESP_LOGI(t, "    bit %3u +%2u  (padding)", f->bit_offset,
                         f->bit_size);
                continue;
            }
            ESP_LOGI(t,
                     "    bit %3u +%2u  page %02x usage %02x %-8s "
                     "range %ld..%ld%s",
                     f->bit_offset, f->bit_size, f->usage_page, f->usage,
                     vx_hid_usage_name(f->usage_page, f->usage),
                     (long)f->logical_min, (long)f->logical_max,
                     (f->flags & VX_HID_FLAG_VARIABLE) ? "" : " [ARRAY]");
        }
    }
}
