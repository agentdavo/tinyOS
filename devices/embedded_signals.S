// SPDX-License-Identifier: MIT OR Apache-2.0
// Embedded EtherCAT signal bindings for the machine registry.

	.section .rodata
	.global _binary_embedded_signals_tsv_start
	.global _binary_embedded_signals_tsv_end
_binary_embedded_signals_tsv_start:
	.incbin "devices/embedded_signals.tsv"
_binary_embedded_signals_tsv_end:
