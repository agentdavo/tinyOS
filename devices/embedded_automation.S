// SPDX-License-Identifier: MIT OR Apache-2.0
// Embedded automation TSVs for macro and ladder runtimes.

	.section .rodata
	.global _binary_embedded_macros_tsv_start
	.global _binary_embedded_macros_tsv_end
_binary_embedded_macros_tsv_start:
	.incbin "devices/embedded_macros.tsv"
_binary_embedded_macros_tsv_end:

	.global _binary_embedded_ladder_tsv_start
	.global _binary_embedded_ladder_tsv_end
_binary_embedded_ladder_tsv_start:
	.incbin "devices/embedded_ladder.tsv"
_binary_embedded_ladder_tsv_end:

	.global _binary_embedded_toolpods_tsv_start
	.global _binary_embedded_toolpods_tsv_end
_binary_embedded_toolpods_tsv_start:
	.incbin "devices/embedded_toolpods.tsv"
_binary_embedded_toolpods_tsv_end:
