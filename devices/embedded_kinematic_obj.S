// SPDX-License-Identifier: MIT OR Apache-2.0
// Embedded OBJ mesh blobs for the machine editor.
//
// When `tools/machine_editor/` exports a bundle, drop each `*.obj` into
// `devices/` and add a matching `.incbin` + global symbol pair here, plus
// a corresponding `RegistryEntry` to `render/obj_registry.cpp`.

	.section .rodata

	.global _binary_demo_box_obj_start
	.global _binary_demo_box_obj_end
_binary_demo_box_obj_start:
	.incbin "devices/demo_box.obj"
_binary_demo_box_obj_end:

	.global _binary_demo_part_stl_start
	.global _binary_demo_part_stl_end
_binary_demo_part_stl_start:
	.incbin "devices/demo_part.stl"
_binary_demo_part_stl_end:
