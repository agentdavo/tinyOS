// SPDX-License-Identifier: MIT OR Apache-2.0
// Embed the binary ESI device-descriptor blob produced by
// tools/esi_to_blob.py. The Makefile's build/esi_payload.bin target runs
// the python tool to (re)generate the blob from ethercat/esi/*.xml before
// this object is assembled, so a fresh checkout still links cleanly.

    .section .rodata, "a"
    .align 3

    .global _binary_esi_payload_start
    .global _binary_esi_payload_end
_binary_esi_payload_start:
    .incbin "build/esi_payload.bin"
_binary_esi_payload_end:
