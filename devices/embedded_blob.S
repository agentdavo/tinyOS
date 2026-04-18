// SPDX-License-Identifier: MIT OR Apache-2.0
// Embed device TSV blobs into the kernel image via .incbin. The linker
// resolves `_binary_clearpath_ec_tsv_start/_end` from these symbols; no
// filesystem and no heap are involved.

    .section .rodata, "a"
    .align 3

    .global _binary_clearpath_ec_tsv_start
    .global _binary_clearpath_ec_tsv_end
_binary_clearpath_ec_tsv_start:
    .incbin "devices/clearpath_ec.tsv"
_binary_clearpath_ec_tsv_end:

    .global _binary_beckhoff_ek_tsv_start
    .global _binary_beckhoff_ek_tsv_end
_binary_beckhoff_ek_tsv_start:
    .incbin "devices/beckhoff_ek.tsv"
_binary_beckhoff_ek_tsv_end:

    // Phase 7 — Beckhoff EL5042 BiSS-C encoder interface for load-side
    // feedback (Renishaw BiSS-C encoders on the axis, closing the outer
    // loop while ClearPath closes the inner motor loop).
    .global _binary_el5042_biss_tsv_start
    .global _binary_el5042_biss_tsv_end
_binary_el5042_biss_tsv_start:
    .incbin "devices/el5042_biss.tsv"
_binary_el5042_biss_tsv_end:

    // General-purpose I/O terminals (Beckhoff standard). All three are
    // byte-packed, so pack/unpack goes via the PdoLayout mechanism.
    .global _binary_el1809_din_tsv_start
    .global _binary_el1809_din_tsv_end
_binary_el1809_din_tsv_start:
    .incbin "devices/el1809_din.tsv"
_binary_el1809_din_tsv_end:

    .global _binary_el2809_dout_tsv_start
    .global _binary_el2809_dout_tsv_end
_binary_el2809_dout_tsv_start:
    .incbin "devices/el2809_dout.tsv"
_binary_el2809_dout_tsv_end:

    .global _binary_el3162_ain_tsv_start
    .global _binary_el3162_ain_tsv_end
_binary_el3162_ain_tsv_start:
    .incbin "devices/el3162_ain.tsv"
_binary_el3162_ain_tsv_end:

    // Beckhoff EL72xx compact-servo family — all three variants share the
    // DS402 profile + same CSP PDO pair. EL7201 has resolver feedback;
    // EL7211/EL7221 have Hiperface DSL (OCT) absolute feedback. Current
    // ratings scale 2.8 / 4.5 / 8.0 A. miniOS drives them through the
    // existing CSP pipeline with no code changes — the TSV picks the PID.
    .global _binary_el7201_9014_servo_tsv_start
    .global _binary_el7201_9014_servo_tsv_end
_binary_el7201_9014_servo_tsv_start:
    .incbin "devices/el7201_9014_servo.tsv"
_binary_el7201_9014_servo_tsv_end:

    .global _binary_el7211_9014_servo_tsv_start
    .global _binary_el7211_9014_servo_tsv_end
_binary_el7211_9014_servo_tsv_start:
    .incbin "devices/el7211_9014_servo.tsv"
_binary_el7211_9014_servo_tsv_end:

    .global _binary_el7221_9014_servo_tsv_start
    .global _binary_el7221_9014_servo_tsv_end
_binary_el7221_9014_servo_tsv_start:
    .incbin "devices/el7221_9014_servo.tsv"
_binary_el7221_9014_servo_tsv_end:
