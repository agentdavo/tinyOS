// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

// Kernel-side WebSocket server. Sits on top of TcpListener; replaces
// the Python bridge in tools/ui_editor/live_preview.py for the live-
// preview path (browser → kernel direct).
//
// Scope (RFC 6455 subset):
//   * Server-only. We accept the client's Upgrade request, do the
//     handshake, then read/write WS frames on the same TCP conn.
//   * Text frames + Close frames only. Binary frames are dropped
//     silently; ping/pong unsupported (no keepalive needed for the
//     short-lived editor sessions).
//   * Client frames MUST be masked per RFC 6455 — we enforce.
//   * Single fragment per message (FIN=1). Continuation frames
//     get dropped (would require streaming reassembly).
//   * One connection per port (inherited from TcpListener).
//
// The user provides a `Handler` that gets the assembled text payload.
// Replies are sent back via WebSocketConnection::send_text(...).

#include "tcp.hpp"

#include <cstddef>
#include <cstdint>

namespace kernel::net {

class WebSocketServer;

class WebSocketConnection {
public:
    bool send_text(const char* text, size_t len) noexcept;
private:
    friend class WebSocketServer;
    TcpConnection* tcp = nullptr;
};

class WebSocketHandler {
public:
    virtual ~WebSocketHandler() = default;
    virtual void on_ws_open(WebSocketConnection& conn) noexcept = 0;
    // Called once per assembled WS text frame. `payload` is UTF-8 and
    // is the unmasked content; the buffer is owned by the WS server
    // and valid only for the duration of the call.
    virtual void on_ws_text(WebSocketConnection& conn,
                            const uint8_t* payload, size_t len) noexcept = 0;
    virtual void on_ws_close(WebSocketConnection& conn) noexcept = 0;
};

class WebSocketServer : public TcpListener {
public:
    // `payload_buf` is the assembly buffer for incoming WS frame
    // payload. Big frames (e.g. a 150 KB TSV upload) need a large
    // buffer; supply one sized to your max expected message. Frames
    // exceeding cap close the connection.
    WebSocketServer(uint16_t port, WebSocketHandler* handler,
                    uint8_t* payload_buf, size_t payload_cap) noexcept;

    uint16_t local_port() const noexcept override { return port_; }
    void on_open(TcpConnection& conn) noexcept override;
    void on_data(TcpConnection& conn,
                 const uint8_t* data, size_t len) noexcept override;
    void on_close(TcpConnection& conn) noexcept override;

private:
    enum class State : uint8_t { Handshake, Frames, Closing };
    enum class FrameStage : uint8_t {
        Header,    // need first 2 bytes
        ExtLen2,   // need 2 more bytes (length code = 126)
        ExtLen8,   // need 8 more bytes (length code = 127)
        Mask,      // need 4 mask bytes
        Payload,   // need `frame_len_` payload bytes
    };

    bool process_handshake(TcpConnection& conn) noexcept;
    void process_frames(TcpConnection& conn,
                        const uint8_t* data, size_t len) noexcept;
    void close_with_status(TcpConnection& conn, uint16_t status) noexcept;

    uint16_t port_;
    WebSocketHandler* handler_;
    State state_ = State::Handshake;
    WebSocketConnection ws_conn_;

    // Handshake parser: HTTP request lines accumulate into http_buf_
    // until \r\n\r\n. 4 KB is generous for the editor's request.
    static constexpr size_t HTTP_BUF_CAP = 4096;
    uint8_t  http_buf_[HTTP_BUF_CAP]{};
    size_t   http_len_ = 0;

    // Frame parser state.
    FrameStage frame_stage_ = FrameStage::Header;
    uint8_t  hdr_buf_[14]{};    // up to 2 + 8 + 4 bytes of frame prefix
    size_t   hdr_len_ = 0;
    bool     frame_fin_ = false;
    uint8_t  frame_opcode_ = 0;
    bool     frame_masked_ = false;
    uint64_t frame_len_ = 0;
    uint8_t  frame_mask_[4]{};
    uint8_t* payload_buf_ = nullptr;
    size_t   payload_cap_ = 0;
    size_t   payload_filled_ = 0;
};

} // namespace kernel::net
