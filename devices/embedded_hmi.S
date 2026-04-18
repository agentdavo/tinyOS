// SPDX-License-Identifier: MIT OR Apache-2.0
// Embedded HMI/IP service configuration.

	.section .rodata
	.global _binary_embedded_hmi_tsv_start
	.global _binary_embedded_hmi_tsv_end
_binary_embedded_hmi_tsv_start:
	.incbin "devices/embedded_hmi.tsv"
_binary_embedded_hmi_tsv_end:
