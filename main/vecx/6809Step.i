{
    unsigned op;
    //unsigned icycles = 0;
    unsigned ea, i0, i1, r;
    stepsDone=icycles=0;
    /*
        if (irq_f) 
        {
            if (((reg_cc & FLAG_F) != FLAG_F)) 
            {
                if (irq_status != IRQ_CWAI) 
                {
                    reg_cc = (reg_cc & ~FLAG_E);
                    inst_psh (0x81, &reg_s, reg_u, &icycles);
                }

                reg_cc = (reg_cc | FLAG_I);
                reg_cc = (reg_cc | FLAG_F);

                reg_pc = read16 (0xfff6);
                irq_status = IRQ_NORMAL;
                icycles += 7;
            } 
            else 
            {
                if (irq_status == IRQ_SYNC) 
                {
                    irq_status = IRQ_NORMAL;
                }
            }
        }
    */
    if (via_ifr & 0x80) 
    {
        if (((reg_cc & FLAG_I) != FLAG_I)) 
        {
            if (irq_status != IRQ_CWAI) 
            {
                reg_cc = (reg_cc | FLAG_E);
                inst_psh (0xff, &reg_s, reg_u, &icycles);
            }

            reg_cc = (reg_cc | FLAG_I);

            reg_pc = read16 (0xfff8);
            irq_status = IRQ_NORMAL;
            icycles += 7;
        } 
        else 
        {
            if (irq_status == IRQ_SYNC) 
            {
                irq_status = IRQ_NORMAL;
            }
        }
    }

    if (irq_status != IRQ_NORMAL) {
        icycles += 1;
        goto end;
    }

    op = pc_read8 ();

    /* computed-goto Dispatch-Tabelle */
    static void *opcode_table[256] = {
        [0 ... 255] = &&op_default,

        [0x00] = &&op_00,
        [0x03] = &&op_03,
        [0x04] = &&op_04,
        [0x06] = &&op_06,
        [0x07] = &&op_07,
        [0x08] = &&op_08,
        [0x09] = &&op_09,
        [0x0a] = &&op_0a,
        [0x0c] = &&op_0c,
        [0x0d] = &&op_0d,
        [0x0e] = &&op_0e,
        [0x0f] = &&op_0f,

        [0x10] = &&op_10,
        [0x11] = &&op_11,
        [0x12] = &&op_12,
        [0x13] = &&op_13,
        [0x16] = &&op_16,
        [0x17] = &&op_17,
        [0x19] = &&op_19,
        [0x1a] = &&op_1a,
        [0x1c] = &&op_1c,
        [0x1d] = &&op_1d,
        [0x1e] = &&op_1e,
        [0x1f] = &&op_1f,

        [0x20] = &&op_20,
        [0x21] = &&op_21,
        [0x22] = &&op_22,
        [0x23] = &&op_23,
        [0x24] = &&op_24,
        [0x25] = &&op_25,
        [0x26] = &&op_26,
        [0x27] = &&op_27,
        [0x28] = &&op_28,
        [0x29] = &&op_29,
        [0x2a] = &&op_2a,
        [0x2b] = &&op_2b,
        [0x2c] = &&op_2c,
        [0x2d] = &&op_2d,
        [0x2e] = &&op_2e,
        [0x2f] = &&op_2f,

        [0x30] = &&op_30,
        [0x31] = &&op_31,
        [0x32] = &&op_32,
        [0x33] = &&op_33,
        [0x34] = &&op_34,
        [0x35] = &&op_35,
        [0x36] = &&op_36,
        [0x37] = &&op_37,

        [0x39] = &&op_39,
        [0x3a] = &&op_3a,
        [0x3b] = &&op_3b,
        [0x3c] = &&op_3c,
        [0x3d] = &&op_3d,
        [0x3f] = &&op_3f,

        [0x40] = &&op_40,
        [0x43] = &&op_43,
        [0x44] = &&op_44,
        [0x46] = &&op_46,
        [0x47] = &&op_47,
        [0x48] = &&op_48,
        [0x49] = &&op_49,
        [0x4a] = &&op_4a,
        [0x4c] = &&op_4c,
        [0x4d] = &&op_4d,
        [0x4f] = &&op_4f,

        [0x50] = &&op_50,
        [0x53] = &&op_53,
        [0x54] = &&op_54,
        [0x56] = &&op_56,
        [0x57] = &&op_57,
        [0x58] = &&op_58,
        [0x59] = &&op_59,
        [0x5a] = &&op_5a,
        [0x5c] = &&op_5c,
        [0x5d] = &&op_5d,
        [0x5f] = &&op_5f,

        [0x60] = &&op_60,
        [0x63] = &&op_63,
        [0x64] = &&op_64,
        [0x66] = &&op_66,
        [0x67] = &&op_67,
        [0x68] = &&op_68,
        [0x69] = &&op_69,
        [0x6a] = &&op_6a,
        [0x6c] = &&op_6c,
        [0x6d] = &&op_6d,
        [0x6e] = &&op_6e,
        [0x6f] = &&op_6f,

        [0x70] = &&op_70,
        [0x73] = &&op_73,
        [0x74] = &&op_74,
        [0x76] = &&op_76,
        [0x77] = &&op_77,
        [0x78] = &&op_78,
        [0x79] = &&op_79,
        [0x7a] = &&op_7a,
        [0x7c] = &&op_7c,
        [0x7d] = &&op_7d,
        [0x7e] = &&op_7e,
        [0x7f] = &&op_7f,

        [0x80] = &&op_80,
        [0x81] = &&op_81,
        [0x82] = &&op_82,
        [0x83] = &&op_83,
        [0x84] = &&op_84,
        [0x85] = &&op_85,
        [0x86] = &&op_86,
        [0x88] = &&op_88,
        [0x89] = &&op_89,
        [0x8a] = &&op_8a,
        [0x8b] = &&op_8b,
        [0x8c] = &&op_8c,
        [0x8d] = &&op_8d,
        [0x8e] = &&op_8e,

        [0x90] = &&op_90,
        [0x91] = &&op_91,
        [0x92] = &&op_92,
        [0x93] = &&op_93,
        [0x94] = &&op_94,
        [0x95] = &&op_95,
        [0x96] = &&op_96,
        [0x97] = &&op_97,
        [0x98] = &&op_98,
        [0x99] = &&op_99,
        [0x9a] = &&op_9a,
        [0x9b] = &&op_9b,
        [0x9c] = &&op_9c,
        [0x9d] = &&op_9d,
        [0x9e] = &&op_9e,
        [0x9f] = &&op_9f,

        [0xa0] = &&op_a0,
        [0xa1] = &&op_a1,
        [0xa2] = &&op_a2,
        [0xa3] = &&op_a3,
        [0xa4] = &&op_a4,
        [0xa5] = &&op_a5,
        [0xa6] = &&op_a6,
        [0xa7] = &&op_a7,
        [0xa8] = &&op_a8,
        [0xa9] = &&op_a9,
        [0xaa] = &&op_aa,
        [0xab] = &&op_ab,
        [0xac] = &&op_ac,
        [0xad] = &&op_ad,
        [0xae] = &&op_ae,
        [0xaf] = &&op_af,

        [0xb0] = &&op_b0,
        [0xb1] = &&op_b1,
        [0xb2] = &&op_b2,
        [0xb3] = &&op_b3,
        [0xb4] = &&op_b4,
        [0xb5] = &&op_b5,
        [0xb6] = &&op_b6,
        [0xb7] = &&op_b7,
        [0xb8] = &&op_b8,
        [0xb9] = &&op_b9,
        [0xba] = &&op_ba,
        [0xbb] = &&op_bb,
        [0xbc] = &&op_bc,
        [0xbd] = &&op_bd,
        [0xbe] = &&op_be,
        [0xbf] = &&op_bf,

        [0xc0] = &&op_c0,
        [0xc1] = &&op_c1,
        [0xc2] = &&op_c2,
        [0xc3] = &&op_c3,
        [0xc4] = &&op_c4,
        [0xc5] = &&op_c5,
        [0xc6] = &&op_c6,
        [0xc8] = &&op_c8,
        [0xc9] = &&op_c9,
        [0xca] = &&op_ca,
        [0xcb] = &&op_cb,
        [0xcc] = &&op_cc,
        [0xce] = &&op_ce,

        [0xd0] = &&op_d0,
        [0xd1] = &&op_d1,
        [0xd2] = &&op_d2,
        [0xd3] = &&op_d3,
        [0xd4] = &&op_d4,
        [0xd5] = &&op_d5,
        [0xd6] = &&op_d6,
        [0xd7] = &&op_d7,
        [0xd8] = &&op_d8,
        [0xd9] = &&op_d9,
        [0xda] = &&op_da,
        [0xdb] = &&op_db,
        [0xdc] = &&op_dc,
        [0xdd] = &&op_dd,
        [0xde] = &&op_de,
        [0xdf] = &&op_df,

        [0xe0] = &&op_e0,
        [0xe1] = &&op_e1,
        [0xe2] = &&op_e2,
        [0xe3] = &&op_e3,
        [0xe4] = &&op_e4,
        [0xe5] = &&op_e5,
        [0xe6] = &&op_e6,
        [0xe7] = &&op_e7,
        [0xe8] = &&op_e8,
        [0xe9] = &&op_e9,
        [0xea] = &&op_ea,
        [0xeb] = &&op_eb,
        [0xec] = &&op_ec,
        [0xed] = &&op_ed,
        [0xee] = &&op_ee,
        [0xef] = &&op_ef,

        [0xf0] = &&op_f0,
        [0xf1] = &&op_f1,
        [0xf2] = &&op_f2,
        [0xf3] = &&op_f3,
        [0xf4] = &&op_f4,
        [0xf5] = &&op_f5,
        [0xf6] = &&op_f6,
        [0xf7] = &&op_f7,
        [0xf8] = &&op_f8,
        [0xf9] = &&op_f9,
        [0xfa] = &&op_fa,
        [0xfb] = &&op_fb,
        [0xfc] = &&op_fc,
        [0xfd] = &&op_fd,
        [0xfe] = &&op_fe,
        [0xff] = &&op_ff,
    };

    goto *opcode_table[op];

/* ---------------- page 0 instructions ---------------- */

op_00: /* 0x00: neg direct */
    ea = ea_direct ();
    r = inst_neg (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_03: /* 0x03: com direct */
    ea = ea_direct ();
    r = inst_com (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_04: /* 0x04: lsr direct */
    ea = ea_direct ();
    r = inst_lsr (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_06: /* 0x06: ror direct */
    ea = ea_direct ();
    r = inst_ror (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_07: /* 0x07: asr direct */
    ea = ea_direct ();
    r = inst_asr (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_08: /* 0x08: asl direct */
    ea = ea_direct ();
    r = inst_asl (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_09: /* 0x09: rol direct */
    ea = ea_direct ();
    r = inst_rol (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_0a: /* 0x0a: dec direct */
    ea = ea_direct ();
    r = inst_dec (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_0c: /* 0x0c: inc direct */
    ea = ea_direct ();
    r = inst_inc (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_0d: /* 0x0d: tst direct */
    ea = ea_direct ();
    inst_tst8 (read8 (ea));
    icycles += 6;
    goto end;

op_0e: /* 0x0e: jmp direct */
    reg_pc = ea_direct ();
    icycles += 3;
    goto end;

op_0f: /* 0x0f: clr direct */
    ea = ea_direct ();
    inst_clr ();
    read8(ea); // clear reads! important for shift reg emulation!
    write8 (ea, 0);
    icycles += 6;
    goto end;

/* page 1 prefix */
op_10: /* 0x10 */
    op = pc_read8 ();
    switch (op) 
    {
        /* lbra / lbrn */
        case 0x20:
        case 0x21:
            inst_bra16 (0, op, &icycles);
            break;

        /* lbhi / lbls */
        case 0x22:
        case 0x23:
            inst_bra16 (get_cc (FLAG_C) | get_cc (FLAG_Z), op, &icycles);
            break;

        /* lbhs/lbcc / lblo/lbcs */
        case 0x24:
        case 0x25:
            inst_bra16 (get_cc (FLAG_C), op, &icycles);
            break;

        /* lbne / lbeq */
        case 0x26:
        case 0x27:
            inst_bra16 (get_cc (FLAG_Z), op, &icycles);
            break;

        /* lbvc / lbvs */
        case 0x28:
        case 0x29:
            inst_bra16 (get_cc (FLAG_V), op, &icycles);
            break;

        /* lbpl / lbmi */
        case 0x2a:
        case 0x2b:
            inst_bra16 (get_cc (FLAG_N), op, &icycles);
            break;

        /* lbge / lblt */
        case 0x2c:
        case 0x2d:
            inst_bra16 (get_cc (FLAG_N) ^ get_cc (FLAG_V), op, &icycles);
            break;

        /* lbgt / lble */
        case 0x2e:
        case 0x2f:
            inst_bra16 (get_cc (FLAG_Z) |
                        (get_cc (FLAG_N) ^ get_cc (FLAG_V)), op, &icycles);
            break;

        /* cmpd */
        case 0x83:
            inst_sub16 (get_reg_d (), pc_read16 ());
            icycles += 5;
            break;
        case 0x93:
            ea = ea_direct ();
            inst_sub16 (get_reg_d (), read16_cycloid (ea));
            icycles += 7;
            break;
        case 0xa3:
            ea = ea_indexed (&icycles);
            inst_sub16 (get_reg_d (), read16_cycloid (ea));
            icycles += 7;
            break;
        case 0xb3:
            ea = ea_extended ();
            inst_sub16 (get_reg_d (), read16_cycloid (ea));
            icycles += 8;
            break;

        /* cmpy */
        case 0x8c:
            inst_sub16 (reg_y, pc_read16 ());
            icycles += 5;
            break;
        case 0x9c:
            ea = ea_direct ();
            inst_sub16 (reg_y, read16 (ea));
            icycles += 7;
            break;
        case 0xac:
            ea = ea_indexed (&icycles);
            inst_sub16 (reg_y, read16 (ea));
            icycles += 7;
            break;
        case 0xbc:
            ea = ea_extended ();
            inst_sub16 (reg_y, read16 (ea));
            icycles += 8;
            break;

        /* ldy */
        case 0x8e:
            reg_y = pc_read16 ();
            inst_tst16 (reg_y);
            icycles += 4;
            break;
        case 0x9e:
            ea = ea_direct ();
            reg_y = read16 (ea);
            inst_tst16 (reg_y);
            icycles += 6;
            break;
        case 0xae:
            ea = ea_indexed (&icycles);
            reg_y = read16 (ea);
            inst_tst16 (reg_y);
            icycles += 6;
            break;
        case 0xbe:
            ea = ea_extended ();
            reg_y = read16 (ea);
            inst_tst16 (reg_y);
            icycles += 7;
            break;

        /* sty */
        case 0x9f:
            ea = ea_direct ();
            write16 (ea, reg_y);
            inst_tst16 (reg_y);
            icycles += 6;
            break;
        case 0xaf:
            ea = ea_indexed (&icycles);
            write16 (ea, reg_y);
            inst_tst16 (reg_y);
            icycles += 6;
            break;
        case 0xbf:
            ea = ea_extended ();
            write16 (ea, reg_y);
            inst_tst16 (reg_y);
            icycles += 7;
            break;

        /* lds */
        case 0xce:
            reg_s = pc_read16 ();
            inst_tst16 (reg_s);
            icycles += 4;
            break;
        case 0xde:
            ea = ea_direct ();
            reg_s = read16 (ea);
            inst_tst16 (reg_s);
            icycles += 6;
            break;
        case 0xee:
            ea = ea_indexed (&icycles);
            reg_s = read16 (ea);
            inst_tst16 (reg_s);
            icycles += 6;
            break;
        case 0xfe:
            ea = ea_extended ();
            reg_s = read16 (ea);
            inst_tst16 (reg_s);
            icycles += 7;
            break;

        /* sts */
        case 0xdf:
            ea = ea_direct ();
            write16 (ea, reg_s);
            inst_tst16 (reg_s);
            icycles += 6;
            break;
        case 0xef:
            ea = ea_indexed (&icycles);
            write16 (ea, reg_s);
            inst_tst16 (reg_s);
            icycles += 6;
            break;
        case 0xff:
            ea = ea_extended ();
            write16 (ea, reg_s);
            inst_tst16 (reg_s);
            icycles += 7;
            break;

        /* swi2 */
        case 0x3f:
            set_cc (FLAG_E, 1);
            inst_psh (0xff, &reg_s, reg_u, &icycles);
            reg_pc = read16 (0xfff4);
            icycles += 8;
            break;

        default:
            break;
    }
    goto end;

/* page 2 prefix */
op_11: /* 0x11 */
    op = pc_read8 ();

    switch (op) {
        /* cmpu */
        case 0x83:
            inst_sub16 (reg_u, pc_read16 ());
            icycles += 5;
            break;
        case 0x93:
            ea = ea_direct ();
            inst_sub16 (reg_u, read16 (ea));
            icycles += 7;
            break;
        case 0xa3:
            ea = ea_indexed (&icycles);
            inst_sub16 (reg_u, read16 (ea));
            icycles += 7;
            break;
        case 0xb3:
            ea = ea_extended ();
            inst_sub16 (reg_u, read16 (ea));
            icycles += 8;
            break;

        /* cmps */
        case 0x8c:
            inst_sub16 (reg_s, pc_read16 ());
            icycles += 5;
            break;
        case 0x9c:
            ea = ea_direct ();
            inst_sub16 (reg_s, read16 (ea));
            icycles += 7;
            break;
        case 0xac:
            ea = ea_indexed (&icycles);
            inst_sub16 (reg_s, read16 (ea));
            icycles += 7;
            break;
        case 0xbc:
            ea = ea_extended ();
            inst_sub16 (reg_s, read16 (ea));
            icycles += 8;
            break;

        /* swi3 */
        case 0x3f:
            set_cc (FLAG_E, 1);
            inst_psh (0xff, &reg_s, reg_u, &icycles);
            reg_pc = read16 (0xfff2);
            icycles += 8;
            break;

        default:
            break;
    }
    goto end;

/* nop */
op_12: /* 0x12 */
    icycles += 2;
    goto end;

/* sync */
op_13: /* 0x13 */
    irq_status = IRQ_SYNC;
    icycles += 2;
    goto end;

/* lbra */
op_16: /* 0x16 */
    r = pc_read16 ();
    reg_pc = (reg_pc + r) & 0xffff;
    icycles += 5;
    goto end;

/* lbsr */
op_17: /* 0x17 */
    r = pc_read16 ();
    push16 (&reg_s, reg_pc);
    reg_pc = (reg_pc + r) & 0xffff;
    icycles += 9;
    goto end;

/* daa */
op_19: /* 0x19 */
    i0 = reg_a;
    i1 = 0;

    if ((reg_a & 0x0f) > 0x09 || ((reg_cc & FLAG_H) == FLAG_H)) {
        i1 |= 0x06;
    }

    if ((reg_a & 0xf0) > 0x80 && (reg_a & 0x0f) > 0x09) {
        i1 |= 0x60;
    }

    if ((reg_a & 0xf0) > 0x90 || ((reg_cc & FLAG_C) == FLAG_C)) {
        i1 |= 0x60;
    }

    reg_a = (i0 + i1) & 0xff;

    set_cc (FLAG_N, test_n (reg_a));
    set_cc (FLAG_Z, test_z8 (reg_a));
    reg_cc = (reg_cc & ~FLAG_V);
    set_cc (FLAG_C, test_c (i0, i1, reg_a, 0));
    icycles += 2;
    goto end;

/* orcc */
op_1a: /* 0x1a */
    reg_cc |= pc_read8 ();
    icycles += 3;
    goto end;

/* andcc */
op_1c: /* 0x1c */
    reg_cc &= pc_read8 ();
    icycles += 3;
    goto end;

/* sex */
op_1d: /* 0x1d */
    set_reg_d (sign_extend (reg_b));
    set_cc (FLAG_N, test_n (reg_a));
    set_cc (FLAG_Z, test_z16 (get_reg_d ()));
    icycles += 2;
    goto end;

/* exg */
op_1e: /* 0x1e */
    inst_exg ();
    icycles += 8;
    goto end;

/* tfr */
op_1f: /* 0x1f */
    inst_tfr ();
    icycles += 6;
    goto end;

/* branches 0x20–0x2f */

op_20: /* 0x20 bra */
op_21: /* 0x21 brn */
    inst_bra8 (0, op, &icycles);
    goto end;

op_22: /* 0x22 bhi */
op_23: /* 0x23 bls */
    inst_bra8 (((reg_cc & FLAG_C) == FLAG_C) | ((reg_cc & FLAG_Z) == FLAG_Z), op, &icycles);
    goto end;

op_24: /* 0x24 bhs/bcc */
op_25: /* 0x25 blo/bcs */
    inst_bra8 (((reg_cc & FLAG_C) == FLAG_C), op, &icycles);
    goto end;

op_26: /* 0x26 bne */
op_27: /* 0x27 beq */
    inst_bra8 (((reg_cc & FLAG_Z) == FLAG_Z), op, &icycles);
    goto end;

op_28: /* 0x28 bvc */
op_29: /* 0x29 bvs */
    inst_bra8 (((reg_cc & FLAG_V) == FLAG_V), op, &icycles);
    goto end;

op_2a: /* 0x2a bpl */
op_2b: /* 0x2b bmi */
    inst_bra8 (((reg_cc & FLAG_N) == FLAG_N), op, &icycles);
    goto end;

op_2c: /* 0x2c bge */
op_2d: /* 0x2d blt */
    inst_bra8 (((reg_cc & FLAG_N) == FLAG_N) ^ ((reg_cc & FLAG_V) == FLAG_V), op, &icycles);
    goto end;

op_2e: /* 0x2e bgt */
op_2f: /* 0x2f ble */
    inst_bra8 (((reg_cc & FLAG_Z) == FLAG_Z) | (((reg_cc & FLAG_N) == FLAG_N) ^ ((reg_cc & FLAG_V) == FLAG_V)), op, &icycles);
    goto end;

/* leax/leay/leas/leau */

op_30: /* 0x30 leax */
    reg_x = ea_indexed (&icycles);
    set_cc (FLAG_Z, test_z16 (reg_x));
    icycles += 4;
    goto end;

op_31: /* 0x31 leay */
    reg_y = ea_indexed (&icycles);
    set_cc (FLAG_Z, test_z16 (reg_y));
    icycles += 4;
    goto end;

op_32: /* 0x32 leas */
    reg_s = ea_indexed (&icycles);
    icycles += 4;
    goto end;

op_33: /* 0x33 leau */
    reg_u = ea_indexed (&icycles);
    icycles += 4;
    goto end;

/* pshs/puls/pshu/pulu */

op_34: /* 0x34 pshs */
    inst_psh (pc_read8 (), &reg_s, reg_u, &icycles);
    icycles += 5;
    goto end;

op_35: /* 0x35 puls */
    inst_pul (pc_read8 (), &reg_s, &reg_u, &icycles);
    icycles += 5;
    goto end;

op_36: /* 0x36 pshu */
    inst_psh (pc_read8 (), &reg_u, reg_s, &icycles);
    icycles += 5;
    goto end;

op_37: /* 0x37 pulu */
    inst_pul (pc_read8 (), &reg_u, &reg_s, &icycles);
    icycles += 5;
    goto end;

/* rts */
op_39: /* 0x39 */
    reg_pc = pull16 (&reg_s);
    icycles += 5;
    goto end;

/* abx */
op_3a: /* 0x3a */
    reg_x += reg_b & 0xff;
    icycles += 3;
    goto end;

/* rti */
op_3b: /* 0x3b */
    inst_pul (0x01, &reg_s, &reg_u, &icycles);
    if (((reg_cc & FLAG_E) == FLAG_E))
    {
        inst_pul (0xfe, &reg_s, &reg_u, &icycles);
    } 
    else 
    {
        inst_pul (0x80, &reg_s, &reg_u, &icycles);
    }
    icycles += 3;
    goto end;

/* cwai */
op_3c: /* 0x3c */
    reg_cc &= pc_read8 ();
    set_cc (FLAG_E, 1);
    inst_psh (0xff, &reg_s, reg_u, &icycles);
    irq_status = IRQ_CWAI;
    icycles += 4;
    goto end;

/* mul */
op_3d: /* 0x3d */
    r = (reg_a & 0xff) * (reg_b & 0xff);
    set_reg_d (r);

    set_cc (FLAG_Z, test_z16 (r));
    set_cc (FLAG_C, (r >> 7) & 1);

    icycles += 11;
    goto end;

/* swi */
op_3f: /* 0x3f */
    set_cc (FLAG_E, 1);
    inst_psh (0xff, &reg_s, reg_u, &icycles);
    set_cc (FLAG_I, 1);
    set_cc (FLAG_F, 1);
    reg_pc = read16 (0xfffa);
    icycles += 7;
    goto end;

/* accumulator / memory ops for A/B, etc. */

/* neg / nega / negb */

op_40: /* 0x40 nega */
    reg_a = inst_neg (reg_a) & 0xff;
    icycles += 2;
    goto end;

op_43: /* 0x43 coma */
    reg_a = inst_com (reg_a);
    icycles += 2;
    goto end;

op_44: /* 0x44 lsra */
    reg_a = inst_lsr (reg_a);
    icycles += 2;
    goto end;

op_46: /* 0x46 rora */
    reg_a = inst_ror (reg_a);
    icycles += 2;
    goto end;

op_47: /* 0x47 asra */
    reg_a = inst_asr (reg_a);
    icycles += 2;
    goto end;

op_48: /* 0x48 asla */
    reg_a = inst_asl (reg_a);
    icycles += 2;
    goto end;

op_49: /* 0x49 rola */
    reg_a = inst_rol (reg_a);
    icycles += 2;
    goto end;

op_4a: /* 0x4a deca */
    reg_a = inst_dec (reg_a);
    icycles += 2;
    goto end;

op_4c: /* 0x4c inca */
    reg_a = inst_inc (reg_a);
    icycles += 2;
    goto end;

op_4d: /* 0x4d tsta */
    inst_tst8 (reg_a);
    icycles += 2;
    goto end;

op_4f: /* 0x4f clra */
    inst_clr ();
    reg_a = 0;
    icycles += 2;
    goto end;

/* B accumulator versions */

op_50: /* 0x50 negb */
    reg_b = inst_neg (reg_b) & 0xff;
    icycles += 2;
    goto end;

op_53: /* 0x53 comb */
    reg_b = inst_com (reg_b);
    icycles += 2;
    goto end;

op_54: /* 0x54 lsrb */
    reg_b = inst_lsr (reg_b);
    icycles += 2;
    goto end;

op_56: /* 0x56 rorb */
    reg_b = inst_ror (reg_b);
    icycles += 2;
    goto end;

op_57: /* 0x57 asrb */
    reg_b = inst_asr (reg_b);
    icycles += 2;
    goto end;

op_58: /* 0x58 aslb */
    reg_b = inst_asl (reg_b);
    icycles += 2;
    goto end;

op_59: /* 0x59 rolb */
    reg_b = inst_rol (reg_b);
    icycles += 2;
    goto end;

op_5a: /* 0x5a decb */
    reg_b = inst_dec (reg_b);
    icycles += 2;
    goto end;

op_5c: /* 0x5c incb */
    reg_b = inst_inc (reg_b);
    icycles += 2;
    goto end;

op_5d: /* 0x5d tstb */
    inst_tst8 (reg_b);
    icycles += 2;
    goto end;

op_5f: /* 0x5f clrb */
    inst_clr ();
    reg_b = 0;
    icycles += 2;
    goto end;

/* indexed/direct/extended versions for many ops... */

/* neg indexed */
op_60: /* 0x60 */
    ea = ea_indexed (&icycles);
    r = inst_neg (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_63: /* 0x63 com indexed */
    ea = ea_indexed (&icycles);
    r = inst_com (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_64: /* 0x64 lsr indexed */
    ea = ea_indexed (&icycles);
    r = inst_lsr (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_66: /* 0x66 ror indexed */
    ea = ea_indexed (&icycles);
    r = inst_ror (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_67: /* 0x67 asr indexed */
    ea = ea_indexed (&icycles);
    r = inst_asr (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_68: /* 0x68 asl indexed */
    ea = ea_indexed (&icycles);
    r = inst_asl (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_69: /* 0x69 rol indexed */
    ea = ea_indexed (&icycles);
    r = inst_rol (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_6a: /* 0x6a dec indexed */
    ea = ea_indexed (&icycles);
    r = inst_dec (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_6c: /* 0x6c inc indexed */
    ea = ea_indexed (&icycles);
    r = inst_inc (read8 (ea));
    write8 (ea, r);
    icycles += 6;
    goto end;

op_6d: /* 0x6d tst indexed */
    ea = ea_indexed (&icycles);
    inst_tst8 (read8 (ea));
    icycles += 6;
    goto end;

op_6e: /* 0x6e jmp indexed */
    reg_pc = ea_indexed (&icycles);
    icycles += 3;
    goto end;

op_6f: /* 0x6f clr indexed */
    ea = ea_indexed (&icycles);
    inst_clr ();
    read8(ea);
    write8 (ea, 0);
    icycles += 6;
    goto end;

/* extended: 0x70–0x7f */

op_70: /* 0x70 neg extended */
    ea = ea_extended ();
    r = inst_neg (read8 (ea));
    write8 (ea, r);
    icycles += 7;
    goto end;

op_73: /* 0x73 com extended */
    ea = ea_extended ();
    r = inst_com (read8 (ea));
    write8 (ea, r);
    icycles += 7;
    goto end;

op_74: /* 0x74 lsr extended */
    ea = ea_extended ();
    r = inst_lsr (read8 (ea));
    write8 (ea, r);
    icycles += 7;
    goto end;

op_76: /* 0x76 ror extended */
    ea = ea_extended ();
    r = inst_ror (read8 (ea));
    write8 (ea, r);
    icycles += 7;
    goto end;

op_77: /* 0x77 asr extended */
    ea = ea_extended ();
    r = inst_asr (read8 (ea));
    write8 (ea, r);
    icycles += 7;
    goto end;

op_78: /* 0x78 asl extended */
    ea = ea_extended ();
    r = inst_asl (read8 (ea));
    write8 (ea, r);
    icycles += 7;
    goto end;

op_79: /* 0x79 rol extended */
    ea = ea_extended ();
    r = inst_rol (read8 (ea));
    write8 (ea, r);
    icycles += 7;
    goto end;

op_7a: /* 0x7a dec extended */
    ea = ea_extended ();
    r = inst_dec (read8 (ea));
    write8 (ea, r);
    icycles += 7;
    goto end;

op_7c: /* 0x7c inc extended */
    ea = ea_extended ();
    r = inst_inc (read8 (ea));
    write8 (ea, r);
    icycles += 7;
    goto end;

op_7d: /* 0x7d tst extended */
    ea = ea_extended ();
    inst_tst8 (read8 (ea));
    icycles += 7;
    goto end;

op_7e: /* 0x7e jmp extended */
    reg_pc = ea_extended ();
    icycles += 4;
    goto end;

op_7f: /* 0x7f clr extended */
    ea = ea_extended ();
    inst_clr ();
    read8(ea);
    write8 (ea, 0);
    icycles += 7;
    goto end;

/* suba */
op_80: /* 0x80 */
    reg_a = inst_sub8 (reg_a, pc_read8 ()) & 0xff;
    icycles += 2;
    goto end;

op_90: /* 0x90 */
    ea = ea_direct ();
    reg_a = inst_sub8 (reg_a, read8 (ea)) & 0xff;
    icycles += 4;
    goto end;

op_a0: /* 0xa0 */
    ea = ea_indexed (&icycles);
    reg_a = inst_sub8 (reg_a, read8 (ea)) & 0xff;
    icycles += 4;
    goto end;

op_b0: /* 0xb0 */
    ea = ea_extended ();
    reg_a = inst_sub8 (reg_a, read8 (ea)) & 0xff;
    icycles += 5;
    goto end;

/* cmpa */
op_81: /* 0x81 */
    inst_sub8 (reg_a, pc_read8 ());
    icycles += 2;
    goto end;

op_91: /* 0x91 */
    ea = ea_direct ();
    inst_sub8 (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_a1: /* 0xa1 */
    ea = ea_indexed (&icycles);
    inst_sub8 (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_b1: /* 0xb1 */
    ea = ea_extended ();
    inst_sub8 (reg_a, read8 (ea));
    icycles += 5;
    goto end;

/* sbca */
op_82: /* 0x82 */
    reg_a = inst_sbc (reg_a, pc_read8 ());
    icycles += 2;
    goto end;

op_92: /* 0x92 */
    ea = ea_direct ();
    reg_a = inst_sbc (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_a2: /* 0xa2 */
    ea = ea_indexed (&icycles);
    reg_a = inst_sbc (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_b2: /* 0xb2 */
    ea = ea_extended ();
    reg_a = inst_sbc (reg_a, read8 (ea));
    icycles += 5;
    goto end;

/* subd */
op_83: /* 0x83 */
    set_reg_d (inst_sub16 (get_reg_d (), pc_read16 ()));
    icycles += 4;
    goto end;

op_93: /* 0x93 */
    ea = ea_direct ();
    set_reg_d (inst_sub16 (get_reg_d (), read16 (ea)));
    icycles += 6;
    goto end;

op_a3: /* 0xa3 */
    ea = ea_indexed (&icycles);
    set_reg_d (inst_sub16 (get_reg_d (), read16 (ea)));
    icycles += 6;
    goto end;

op_b3: /* 0xb3 */
    ea = ea_extended ();
    set_reg_d (inst_sub16 (get_reg_d (), read16 (ea)));
    icycles += 7;
    goto end;

/* anda */
op_84: /* 0x84 */
    reg_a = inst_and (reg_a, pc_read8 ());
    icycles += 2;
    goto end;

op_94: /* 0x94 */
    ea = ea_direct ();
    reg_a = inst_and (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_a4: /* 0xa4 */
    ea = ea_indexed (&icycles);
    reg_a = inst_and (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_b4: /* 0xb4 */
    ea = ea_extended ();
    reg_a = inst_and (reg_a, read8 (ea));
    icycles += 5;
    goto end;

/* bita */
op_85: /* 0x85 */
    inst_and (reg_a, pc_read8 ());
    icycles += 2;
    goto end;

op_95: /* 0x95 */
		ea = ea_direct ();
		vecx_intermediateSteps(3); // MUST KEEEP FOR BEDLAM
		inst_and (reg_a, read8 (ea));
//		vecx_intermediateSteps(1);
		icycles += 4;
    goto end;

op_a5: /* 0xa5 */
    ea = ea_indexed (&icycles);
    inst_and (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_b5: /* 0xb5 */
    ea = ea_extended ();
    inst_and (reg_a, read8 (ea));
    icycles += 5;
    goto end;

/* lda */
op_86: /* 0x86 */
    dataBUS = reg_a = pc_read8 ();
    inst_tst8 (reg_a);
    icycles += 2;
    goto end;

op_96: /* 0x96 */
    ea = ea_direct ();
    reg_a = read8 (ea);
    inst_tst8 (reg_a);
    icycles += 4;
    goto end;

op_a6: /* 0xa6 */
    ea = ea_indexed (&icycles);
    reg_a = read8 (ea);
    inst_tst8 (reg_a);
    icycles += 4;
    goto end;

op_b6: /* 0xb6 */
    ea = ea_extended ();
    reg_a = read8 (ea);
    inst_tst8 (reg_a);
    icycles += 5;
    goto end;

/* bsr */
op_8d: /* 0x8d */
    r = pc_read8 ();
    push16 (&reg_s, reg_pc);
    reg_pc = (reg_pc + sign_extend (r)) & 0xffff;
    icycles += 7;
    goto end;

/* cmpx */
op_8c: /* 0x8c */
    inst_sub16 (reg_x, pc_read16 ());
    icycles += 4;
    goto end;

op_9c: /* 0x9c */
    ea = ea_direct ();
    inst_sub16 (reg_x, read16 (ea));
    icycles += 6;
    goto end;

op_ac: /* 0xac */
    ea = ea_indexed (&icycles);
    inst_sub16 (reg_x, read16 (ea));
    icycles += 6;
    goto end;

op_bc: /* 0xbc */
    ea = ea_extended ();
    inst_sub16 (reg_x, read16 (ea));
    icycles += 7;
    goto end;

/* ldx */
op_8e: /* 0x8e */
    reg_x = pc_read16 ();
    inst_tst16 (reg_x);
    icycles += 3;
    goto end;

op_9e: /* 0x9e */
    ea = ea_direct ();
    reg_x = read16 (ea);
    inst_tst16 (reg_x);
    icycles += 5;
    goto end;

op_ae: /* 0xae */
    ea = ea_indexed (&icycles);
    reg_x = read16 (ea);
    inst_tst16 (reg_x);
    icycles += 5;
    goto end;

op_be: /* 0xbe */
    ea = ea_extended ();
    reg_x = read16 (ea);
    inst_tst16 (reg_x);
    icycles += 6;
    goto end;

/* jsr */
op_9d: /* 0x9d */
    ea = ea_direct ();
    push16 (&reg_s, reg_pc);
    reg_pc = ea;
    icycles += 7;
    goto end;

op_ad: /* 0xad */
    ea = ea_indexed (&icycles);
    push16 (&reg_s, reg_pc);
    reg_pc = ea;
    icycles += 7;
    goto end;

op_bd: /* 0xbd */
    ea = ea_extended ();
    push16 (&reg_s, reg_pc);
    reg_pc = ea;
    icycles += 8;
    goto end;

/* sta */
op_97: /* 0x97 */
    ea = ea_direct ();
    write8 (ea, reg_a);
    inst_tst8 (reg_a);
    icycles += 4;
    goto end;

op_a7: /* 0xa7 */
    ea = ea_indexed (&icycles);
    write8 (ea, reg_a);
    inst_tst8 (reg_a);
    icycles += 4;
    goto end;

op_b7: /* 0xb7 */
    ea = ea_extended ();
    write8 (ea, reg_a);
    inst_tst8 (reg_a);
    icycles += 5;
    goto end;

/* stb */
op_d7: /* 0xd7 */
    ea = ea_direct ();
    write8 (ea, reg_b);
    inst_tst8 (reg_b);
    icycles += 4;
    goto end;

op_e7: /* 0xe7 */
    ea = ea_indexed (&icycles);
    write8 (ea, reg_b);
    inst_tst8 (reg_b);
    icycles += 4;
    goto end;

op_f7: /* 0xf7 */
    ea = ea_extended ();
    write8 (ea, reg_b);
    inst_tst8 (reg_b);
    icycles += 5;
    goto end;

/* eora */
op_88: /* 0x88 */
    reg_a = inst_eor (reg_a, pc_read8 ());
    icycles += 2;
    goto end;

op_98: /* 0x98 */
    ea = ea_direct ();
    reg_a = inst_eor (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_a8: /* 0xa8 */
    ea = ea_indexed (&icycles);
    reg_a = inst_eor (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_b8: /* 0xb8 */
    ea = ea_extended ();
    reg_a = inst_eor (reg_a, read8 (ea));
    icycles += 5;
    goto end;

/* adca */
op_89: /* 0x89 */
    reg_a = inst_adc (reg_a, pc_read8 ());
    icycles += 2;
    goto end;

op_99: /* 0x99 */
    ea = ea_direct ();
    reg_a = inst_adc (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_a9: /* 0xa9 */
    ea = ea_indexed (&icycles);
    reg_a = inst_adc (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_b9: /* 0xb9 */
    ea = ea_extended ();
    reg_a = inst_adc (reg_a, read8 (ea));
    icycles += 5;
    goto end;

/* ora */
op_8a: /* 0x8a */
    reg_a = inst_or (reg_a, pc_read8 ());
    icycles += 2;
    goto end;

op_9a: /* 0x9a */
    ea = ea_direct ();
    reg_a = inst_or (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_aa: /* 0xaa */
    ea = ea_indexed (&icycles);
    reg_a = inst_or (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_ba: /* 0xba */
    ea = ea_extended ();
    reg_a = inst_or (reg_a, read8 (ea));
    icycles += 5;
    goto end;

/* adda */
op_8b: /* 0x8b */
    reg_a = inst_add8 (reg_a, pc_read8 ());
    icycles += 2;
    goto end;

op_9b: /* 0x9b */
    ea = ea_direct ();
    reg_a = inst_add8 (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_ab: /* 0xab */
    ea = ea_indexed (&icycles);
    reg_a = inst_add8 (reg_a, read8 (ea));
    icycles += 4;
    goto end;

op_bb: /* 0xbb */
    ea = ea_extended ();
    reg_a = inst_add8 (reg_a, read8 (ea));
    icycles += 5;
    goto end;

/* ldd */
op_cc: /* 0xcc */
    set_reg_d (pc_read16 ());
    inst_tst16 (get_reg_d ());
    icycles += 3;
    goto end;

op_dc: /* 0xdc */
    ea = ea_direct ();
    set_reg_d (read16_cycloid (ea));
    inst_tst16 (get_reg_d ());
    icycles += 5;
    goto end;

op_ec: /* 0xec */
    ea = ea_indexed (&icycles);
    set_reg_d (read16_cycloid (ea));
    inst_tst16 (get_reg_d ());
    icycles += 5;
    goto end;

op_fc: /* 0xfc */
    ea = ea_extended ();
    set_reg_d (read16_cycloid (ea));
    inst_tst16 (get_reg_d ());
    icycles += 6;
    goto end;

/* cmpb */
op_c1: /* 0xc1 */
    inst_sub8 (reg_b, pc_read8 ());
    icycles += 2;
    goto end;

op_d1: /* 0xd1 */
    ea = ea_direct ();
    inst_sub8 (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_e1: /* 0xe1 */
    ea = ea_indexed (&icycles);
    inst_sub8 (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_f1: /* 0xf1 */
    ea = ea_extended ();
    inst_sub8 (reg_b, read8 (ea));
    icycles += 5;
    goto end;

/* sbcb */
op_c2: /* 0xc2 */
    reg_b = inst_sbc (reg_b, pc_read8 ());
    icycles += 2;
    goto end;

op_d2: /* 0xd2 */
    ea = ea_direct ();
    reg_b = inst_sbc (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_e2: /* 0xe2 */
    ea = ea_indexed (&icycles);
    reg_b = inst_sbc (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_f2: /* 0xf2 */
    ea = ea_extended ();
    reg_b = inst_sbc (reg_b, read8 (ea));
    icycles += 5;
    goto end;

/* addd */
op_c3: /* 0xc3 */
    set_reg_d (inst_add16 (get_reg_d (), pc_read16 ()));
    icycles += 4;
    goto end;

op_d3: /* 0xd3 */
    ea = ea_direct ();
    set_reg_d (inst_add16 (get_reg_d (), read16 (ea)));
    icycles += 6;
    goto end;

op_e3: /* 0xe3 */
    ea = ea_indexed (&icycles);
    set_reg_d (inst_add16 (get_reg_d (), read16 (ea)));
    icycles += 6;
    goto end;

op_f3: /* 0xf3 */
    ea = ea_extended ();
    set_reg_d (inst_add16 (get_reg_d (), read16 (ea)));
    icycles += 7;
    goto end;

/* andb */
op_c4: /* 0xc4 */
    reg_b = inst_and (reg_b, pc_read8 ());
    icycles += 2;
    goto end;

op_d4: /* 0xd4 */
    ea = ea_direct ();
    reg_b = inst_and (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_e4: /* 0xe4 */
    ea = ea_indexed (&icycles);
    reg_b = inst_and (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_f4: /* 0xf4 */
    ea = ea_extended ();
    reg_b = inst_and (reg_b, read8 (ea));
    icycles += 5;
    goto end;

/* bitb */
op_c5: /* 0xc5 */
    inst_and (reg_b, pc_read8 ());
    icycles += 2;
    goto end;

op_d5: /* 0xd5 */
    ea = ea_direct ();
    inst_and (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_e5: /* 0xe5 */
    ea = ea_indexed (&icycles);
    inst_and (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_f5: /* 0xf5 */
    ea = ea_extended ();
    inst_and (reg_b, read8 (ea));
    icycles += 5;
    goto end;

/* ldb */
op_c6: /* 0xc6 */
    dataBUS = reg_b = pc_read8 ();
    inst_tst8 (reg_b);
    icycles += 2;
    goto end;

op_d6: /* 0xd6 */
    ea = ea_direct ();
    reg_b = read8 (ea);
    inst_tst8 (reg_b);
    icycles += 4;
    goto end;

op_e6: /* 0xe6 */
    ea = ea_indexed (&icycles);
    reg_b = read8 (ea);
    inst_tst8 (reg_b);
    icycles += 4;
    goto end;

op_f6: /* 0xf6 */
    ea = ea_extended ();
    reg_b = read8 (ea);
    inst_tst8 (reg_b);
    icycles += 5;
    goto end;

/* eorb */
op_c8: /* 0xc8 */
    reg_b = inst_eor (reg_b, pc_read8 ());
    icycles += 2;
    goto end;

op_d8: /* 0xd8 */
    ea = ea_direct ();
    reg_b = inst_eor (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_e8: /* 0xe8 */
    ea = ea_indexed (&icycles);
    reg_b = inst_eor (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_f8: /* 0xf8 */
    ea = ea_extended ();
    reg_b = inst_eor (reg_b, read8 (ea));
    icycles += 5;
    goto end;

/* adcb */
op_c9: /* 0xc9 */
    reg_b = inst_adc (reg_b, pc_read8 ());
    icycles += 2;
    goto end;

op_d9: /* 0xd9 */
    ea = ea_direct ();
    reg_b = inst_adc (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_e9: /* 0xe9 */
    ea = ea_indexed (&icycles);
    reg_b = inst_adc (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_f9: /* 0xf9 */
    ea = ea_extended ();
    reg_b = inst_adc (reg_b, read8 (ea));
    icycles += 5;
    goto end;

/* orb */
op_ca: /* 0xca */
    reg_b = inst_or (reg_b, pc_read8 ());
    icycles += 2;
    goto end;

op_da: /* 0xda */
    ea = ea_direct ();
    reg_b = inst_or (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_ea: /* 0xea */
    ea = ea_indexed (&icycles);
    reg_b = inst_or (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_fa: /* 0xfa */
    ea = ea_extended ();
    reg_b = inst_or (reg_b, read8 (ea));
    icycles += 5;
    goto end;

/* addb */
op_cb: /* 0xcb */
    reg_b = inst_add8 (reg_b, pc_read8 ());
    icycles += 2;
    goto end;

op_db: /* 0xdb */
    ea = ea_direct ();
    reg_b = inst_add8 (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_eb: /* 0xeb */
    ea = ea_indexed (&icycles);
    reg_b = inst_add8 (reg_b, read8 (ea));
    icycles += 4;
    goto end;

op_fb: /* 0xfb */
    ea = ea_extended ();
    reg_b = inst_add8 (reg_b, read8 (ea));
    icycles += 5;
    goto end;

/* ldu */
op_ce: /* 0xce */
    reg_u = pc_read16 ();
    inst_tst16 (reg_u);
    icycles += 3;
    goto end;

op_de: /* 0xde */
    ea = ea_direct ();
    reg_u = read16 (ea);
    inst_tst16 (reg_u);
    icycles += 5;
    goto end;

op_ee: /* 0xee */
    ea = ea_indexed (&icycles);
    reg_u = read16 (ea);
    inst_tst16 (reg_u);
    icycles += 5;
    goto end;

op_fe: /* 0xfe */
    ea = ea_extended ();
    reg_u = read16 (ea);
    inst_tst16 (reg_u);
    icycles += 6;
    goto end;

/* stx */
op_9f: /* 0x9f */
    ea = ea_direct ();
    write16_cycloid (ea, reg_x);
    inst_tst16 (reg_x);
    icycles += 5;
    goto end;

op_af: /* 0xaf */
    ea = ea_indexed (&icycles);
    write16_cycloid (ea, reg_x);
    inst_tst16 (reg_x);
    icycles += 5;
    goto end;

op_bf: /* 0xbf */
    ea = ea_extended ();
    write16_cycloid (ea, reg_x);
    inst_tst16 (reg_x);
    icycles += 6;
    goto end;

/* stu */
op_df: /* 0xdf */
    ea = ea_direct ();
    write16_cycloid (ea, reg_u);
    inst_tst16 (reg_u);
    icycles += 5;
    goto end;

op_ef: /* 0xef */
    ea = ea_indexed (&icycles);
    write16_cycloid (ea, reg_u);
    inst_tst16 (reg_u);
    icycles += 5;
    goto end;

op_ff: /* 0xff */
    ea = ea_extended ();
    write16_cycloid (ea, reg_u);
    inst_tst16 (reg_u);
    icycles += 6;
    goto end;

/* std */
op_dd: /* 0xdd */
    ea = ea_direct ();
    write16_cycloid (ea, get_reg_d ());
    inst_tst16 (get_reg_d ());
    icycles += 5;
    goto end;

op_ed: /* 0xed */
    ea = ea_indexed (&icycles);
    write16_cycloid (ea, get_reg_d ());
    inst_tst16 (get_reg_d ());
    icycles += 5;
    goto end;

op_fd: /* 0xfd */
    ea = ea_extended ();
    write16_cycloid (ea, get_reg_d ());
    inst_tst16 (get_reg_d ());
    icycles += 6;
    goto end;

/* subb */
op_c0: /* 0xC0 */
    reg_b = inst_sub8 (reg_b, pc_read8 ()) & 0xff;
    icycles += 2;
    goto end;

op_d0: /* 0xD0 */
    ea = ea_direct ();
    reg_b = inst_sub8 (reg_b, read8 (ea)) & 0xff;
    icycles += 4;
    goto end;

op_e0: /* 0xE0 */
    ea = ea_indexed (&icycles);
    reg_b = inst_sub8 (reg_b, read8 (ea)) & 0xff;
    icycles += 4;
    goto end;

op_f0: /* 0xF0 */
    ea = ea_extended ();
    reg_b = inst_sub8 (reg_b, read8 (ea)) & 0xff;
    icycles += 5;
    goto end;


/* default / illegal opcode */
op_default:
    /* keine Aktion, nur Zyklen behalten */
    goto end;

end:
}