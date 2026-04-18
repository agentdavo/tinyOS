// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

namespace kernel::ui {

// Configure virtio-gpu scanout + input backend. Safe to call before the
// scheduler starts so the QEMU display window activates during boot even
// if the UI thread hasn't been scheduled yet.
void init_ui_backends();

void boot_ui_once();
void render_ui_once();

// Low-priority UI thread. Runs the TSV-backed UI main loop and refreshes
// it periodically. Assumes init_ui_backends() + boot_ui_once() already ran
// on the boot path.
void boot_ui_thread_entry(void* arg);

}
