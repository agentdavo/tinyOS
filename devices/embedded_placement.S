// SPDX-License-Identifier: MIT OR Apache-2.0
// Embedded runtime placement TSV.

	.section .rodata
	.global _binary_embedded_placement_tsv_start
	.global _binary_embedded_placement_tsv_end
_binary_embedded_placement_tsv_start:
	.incbin "devices/embedded_placement.tsv"
_binary_embedded_placement_tsv_end:
