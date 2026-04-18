// SPDX-License-Identifier: MIT OR Apache-2.0
// Embedded kinematic TSV definitions for the live GLES1 machine preview.
// The source files live in `machines/` alongside the machine editor; this
// file .incbins them into `.rodata` so the kernel has sensible defaults
// when booting without an SD card. When the SD-backed VFS mounts, its
// entries shadow these on path collision.

	.section .rodata

	.global _binary_kinematic_mill3_tsv_start
	.global _binary_kinematic_mill3_tsv_end
_binary_kinematic_mill3_tsv_start:
	.incbin "machines/kinematic_mill3.tsv"
_binary_kinematic_mill3_tsv_end:

	.global _binary_kinematic_millturn_tsv_start
	.global _binary_kinematic_millturn_tsv_end
_binary_kinematic_millturn_tsv_start:
	.incbin "machines/kinematic_millturn.tsv"
_binary_kinematic_millturn_tsv_end:

	.global _binary_kinematic_mx850_tsv_start
	.global _binary_kinematic_mx850_tsv_end
_binary_kinematic_mx850_tsv_start:
	.incbin "machines/kinematic_mx850.tsv"
_binary_kinematic_mx850_tsv_end:
