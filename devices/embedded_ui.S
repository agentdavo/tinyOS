// SPDX-License-Identifier: MIT OR Apache-2.0
// Embedded UI definition - parsed at boot by ui_builder_tsv.cpp
// Override by replacing embedded_ui.tsv and rebuilding

	.section .rodata
	.global _binary_embedded_ui_tsv_start
	.global _binary_embedded_ui_tsv_end
_binary_embedded_ui_tsv_start:
	.incbin "devices/embedded_ui.tsv"
_binary_embedded_ui_tsv_end: