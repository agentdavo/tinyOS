// SPDX-License-Identifier: MIT OR Apache-2.0
// Embedded EtherCAT topology bindings for motion and bus-role ownership.

	.section .rodata
	.global _binary_embedded_topology_tsv_start
	.global _binary_embedded_topology_tsv_end
_binary_embedded_topology_tsv_start:
	.incbin "devices/embedded_topology.tsv"
_binary_embedded_topology_tsv_end:
