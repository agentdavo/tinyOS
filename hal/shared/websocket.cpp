// SPDX-License-Identifier: MIT OR Apache-2.0
//
// WebSocket server — see websocket.hpp for the scope. Hand-rolls SHA-1
// + base64 (16-line each) to stay freestanding. The implementation is
// stream-driven: TCP segments arrive in on_data, the parser pumps
// bytes through its state machine and surfaces complete WS text
// frames to the handler.

#include "websocket.hpp"

#include <cstring>

namespace kernel::net {

namespace {

constexpr const char* WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

// --- SHA-1 (RFC 3174 reference). 20-byte digest, no streaming API
// since our input fits in a single block easily — 60 bytes of key+GUID
// padded to 64 bytes is one block.

void sha1_block(uint32_t h[5], const uint8_t* block) noexcept {
    uint32_t w[80];
    for (int i = 0; i < 16; ++i) {
        w[i] = (static_cast<uint32_t>(block[i * 4]) << 24) |
               (static_cast<uint32_t>(block[i * 4 + 1]) << 16) |
               (static_cast<uint32_t>(block[i * 4 + 2]) << 8) |
                static_cast<uint32_t>(block[i * 4 + 3]);
    }
    for (int i = 16; i < 80; ++i) {
        uint32_t x = w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16];
        w[i] = (x << 1) | (x >> 31);
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    for (int i = 0; i < 80; ++i) {
        uint32_t f, k;
        if (i < 20)      { f = (b & c) | ((~b) & d);   k = 0x5A827999u; }
        else if (i < 40) { f = b ^ c ^ d;              k = 0x6ED9EBA1u; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDCu; }
        else             { f = b ^ c ^ d;              k = 0xCA62C1D6u; }
        uint32_t t = ((a << 5) | (a >> 27)) + f + e + k + w[i];
        e = d; d = c; c = (b << 30) | (b >> 2); b = a; a = t;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
}

void sha1(const uint8_t* msg, size_t len, uint8_t out[20]) noexcept {
    uint32_t h[5] = { 0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u };
    // Pad-and-block — for our use the message is ~60 bytes, so 1 or 2
    // blocks max. General implementation here.
    uint8_t block[64];
    size_t i = 0;
    while (len - i >= 64) {
        sha1_block(h, msg + i);
        i += 64;
    }
    size_t remaining = len - i;
    std::memcpy(block, msg + i, remaining);
    block[remaining] = 0x80;
    if (remaining + 1 + 8 > 64) {
        std::memset(block + remaining + 1, 0, 64 - remaining - 1);
        sha1_block(h, block);
        std::memset(block, 0, 56);
    } else {
        std::memset(block + remaining + 1, 0, 64 - remaining - 1 - 8);
    }
    uint64_t bits = static_cast<uint64_t>(len) * 8u;
    for (int b = 0; b < 8; ++b) block[56 + b] = static_cast<uint8_t>(bits >> (56 - 8 * b));
    sha1_block(h, block);
    for (int j = 0; j < 5; ++j) {
        out[j * 4]     = static_cast<uint8_t>(h[j] >> 24);
        out[j * 4 + 1] = static_cast<uint8_t>(h[j] >> 16);
        out[j * 4 + 2] = static_cast<uint8_t>(h[j] >> 8);
        out[j * 4 + 3] = static_cast<uint8_t>(h[j]);
    }
}

// --- base64 encode (RFC 4648). Output 4 chars per 3 bytes, '=' pad.
size_t base64_encode(const uint8_t* in, size_t in_len, char* out) noexcept {
    static const char alphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t i = 0, j = 0;
    while (i + 3 <= in_len) {
        uint32_t v = (static_cast<uint32_t>(in[i]) << 16) |
                     (static_cast<uint32_t>(in[i + 1]) << 8) |
                      static_cast<uint32_t>(in[i + 2]);
        out[j++] = alphabet[(v >> 18) & 0x3F];
        out[j++] = alphabet[(v >> 12) & 0x3F];
        out[j++] = alphabet[(v >> 6) & 0x3F];
        out[j++] = alphabet[v & 0x3F];
        i += 3;
    }
    if (i < in_len) {
        uint32_t v = static_cast<uint32_t>(in[i]) << 16;
        if (i + 1 < in_len) v |= static_cast<uint32_t>(in[i + 1]) << 8;
        out[j++] = alphabet[(v >> 18) & 0x3F];
        out[j++] = alphabet[(v >> 12) & 0x3F];
        out[j++] = (i + 1 < in_len) ? alphabet[(v >> 6) & 0x3F] : '=';
        out[j++] = '=';
    }
    return j;
}

// Case-insensitive token search inside an HTTP request header buffer.
// Returns a pointer to the *value* (after the colon and any whitespace)
// for header `name`, or nullptr if not present. `end` is exclusive.
const char* find_header(const uint8_t* buf, size_t len, const char* name) noexcept {
    const size_t name_len = std::strlen(name);
    // Walk line by line.
    for (size_t i = 0; i < len; ) {
        size_t eol = i;
        while (eol < len && buf[eol] != '\n') ++eol;
        // Line is [i, eol). Strip trailing \r.
        size_t line_end = eol;
        if (line_end > i && buf[line_end - 1] == '\r') --line_end;
        if (line_end - i > name_len + 1) {
            bool match = true;
            for (size_t k = 0; k < name_len; ++k) {
                char a = static_cast<char>(buf[i + k]);
                char b = name[k];
                if (a >= 'A' && a <= 'Z') a = static_cast<char>(a - 'A' + 'a');
                if (b >= 'A' && b <= 'Z') b = static_cast<char>(b - 'A' + 'a');
                if (a != b) { match = false; break; }
            }
            if (match && buf[i + name_len] == ':') {
                size_t v = i + name_len + 1;
                while (v < line_end && (buf[v] == ' ' || buf[v] == '\t')) ++v;
                return reinterpret_cast<const char*>(buf + v);
            }
        }
        i = eol + 1;
    }
    return nullptr;
}

} // namespace

bool WebSocketConnection::send_text(const char* text, size_t len) noexcept {
    if (!tcp || tcp->state != TcpState::Established) return false;
    // Server frames are never masked. Single-fragment text.
    uint8_t header[10];
    size_t hlen = 0;
    header[hlen++] = 0x81;  // FIN | text
    if (len < 126) {
        header[hlen++] = static_cast<uint8_t>(len);
    } else if (len < 65536) {
        header[hlen++] = 126;
        header[hlen++] = static_cast<uint8_t>(len >> 8);
        header[hlen++] = static_cast<uint8_t>(len);
    } else {
        header[hlen++] = 127;
        for (int s = 56; s >= 0; s -= 8) {
            header[hlen++] = static_cast<uint8_t>(len >> s);
        }
    }
    // Stage header + payload into a single buffer so TcpConnection::send
    // produces one TCP segment (better than two segments back-to-back
    // for clients that don't coalesce). Cap at our staging size.
    static uint8_t frame[2048];
    if (hlen + len > sizeof(frame)) {
        // Two-segment fallback for big payloads.
        if (!tcp->send(header, hlen)) return false;
        return tcp->send(reinterpret_cast<const uint8_t*>(text), len);
    }
    std::memcpy(frame, header, hlen);
    std::memcpy(frame + hlen, text, len);
    return tcp->send(frame, hlen + len);
}

WebSocketServer::WebSocketServer(uint16_t port, WebSocketHandler* handler,
                                 uint8_t* payload_buf, size_t payload_cap) noexcept
    : port_(port), handler_(handler),
      payload_buf_(payload_buf), payload_cap_(payload_cap) {}

void WebSocketServer::on_open(TcpConnection& conn) noexcept {
    state_ = State::Handshake;
    http_len_ = 0;
    frame_stage_ = FrameStage::Header;
    hdr_len_ = 0;
    payload_filled_ = 0;
    ws_conn_.tcp = &conn;
}

void WebSocketServer::on_close(TcpConnection& conn) noexcept {
    (void)conn;
    if (state_ == State::Frames && handler_) handler_->on_ws_close(ws_conn_);
    state_ = State::Closing;
    ws_conn_.tcp = nullptr;
}

void WebSocketServer::on_data(TcpConnection& conn,
                              const uint8_t* data, size_t len) noexcept {
    if (state_ == State::Handshake) {
        // Accumulate request into http_buf_ until \r\n\r\n.
        size_t copy = len;
        if (http_len_ + copy > HTTP_BUF_CAP) copy = HTTP_BUF_CAP - http_len_;
        std::memcpy(http_buf_ + http_len_, data, copy);
        http_len_ += copy;
        // Find end of headers.
        bool found = false;
        for (size_t i = 3; i < http_len_; ++i) {
            if (http_buf_[i - 3] == '\r' && http_buf_[i - 2] == '\n' &&
                http_buf_[i - 1] == '\r' && http_buf_[i]     == '\n') {
                found = true;
                break;
            }
        }
        if (!found) {
            if (http_len_ >= HTTP_BUF_CAP) close_with_status(conn, 1009);
            return;
        }
        if (!process_handshake(conn)) {
            close_with_status(conn, 1002);
            return;
        }
        state_ = State::Frames;
        if (handler_) handler_->on_ws_open(ws_conn_);
        // Any bytes past the handshake belong to the first frame.
        const size_t consumed = copy;  // we copied this many into http_buf_
        if (len > consumed) {
            // Already-buffered data beyond the handshake — unlikely
            // for normal browsers but possible if pipelined. Push
            // through the frame parser.
            process_frames(conn, data + consumed, len - consumed);
        }
        return;
    }
    if (state_ == State::Frames) {
        process_frames(conn, data, len);
    }
}

bool WebSocketServer::process_handshake(TcpConnection& conn) noexcept {
    const char* key_v = find_header(http_buf_, http_len_, "sec-websocket-key");
    if (!key_v) return false;
    // Key is up to end-of-line.
    const char* key_end = key_v;
    while (key_end < reinterpret_cast<const char*>(http_buf_ + http_len_) &&
           *key_end != '\r' && *key_end != '\n') ++key_end;
    const size_t key_len = static_cast<size_t>(key_end - key_v);
    if (key_len == 0 || key_len > 64) return false;

    // Compute accept = base64(sha1(key + GUID)).
    uint8_t concat[128];
    if (key_len + 36 > sizeof(concat)) return false;
    std::memcpy(concat, key_v, key_len);
    std::memcpy(concat + key_len, WS_GUID, 36);
    uint8_t digest[20];
    sha1(concat, key_len + 36, digest);
    char accept[32];
    const size_t accept_len = base64_encode(digest, 20, accept);

    // Build response.
    static const char resp_prefix[] =
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: ";
    static const char resp_suffix[] = "\r\n\r\n";
    static uint8_t resp[256];
    size_t off = 0;
    std::memcpy(resp + off, resp_prefix, sizeof(resp_prefix) - 1); off += sizeof(resp_prefix) - 1;
    std::memcpy(resp + off, accept, accept_len);                   off += accept_len;
    std::memcpy(resp + off, resp_suffix, sizeof(resp_suffix) - 1); off += sizeof(resp_suffix) - 1;
    return conn.send(resp, off);
}

void WebSocketServer::process_frames(TcpConnection& conn,
                                     const uint8_t* data, size_t len) noexcept {
    size_t i = 0;
    while (i < len) {
        switch (frame_stage_) {
            case FrameStage::Header: {
                while (hdr_len_ < 2 && i < len) hdr_buf_[hdr_len_++] = data[i++];
                if (hdr_len_ < 2) return;
                frame_fin_     = (hdr_buf_[0] & 0x80) != 0;
                frame_opcode_  = hdr_buf_[0] & 0x0F;
                frame_masked_  = (hdr_buf_[1] & 0x80) != 0;
                const uint8_t llen = hdr_buf_[1] & 0x7F;
                if (llen < 126) {
                    frame_len_ = llen;
                    frame_stage_ = frame_masked_ ? FrameStage::Mask : FrameStage::Payload;
                } else if (llen == 126) {
                    frame_stage_ = FrameStage::ExtLen2;
                } else {
                    frame_stage_ = FrameStage::ExtLen8;
                }
                break;
            }
            case FrameStage::ExtLen2: {
                while (hdr_len_ < 4 && i < len) hdr_buf_[hdr_len_++] = data[i++];
                if (hdr_len_ < 4) return;
                frame_len_ = (static_cast<uint64_t>(hdr_buf_[2]) << 8) | hdr_buf_[3];
                frame_stage_ = frame_masked_ ? FrameStage::Mask : FrameStage::Payload;
                break;
            }
            case FrameStage::ExtLen8: {
                while (hdr_len_ < 10 && i < len) hdr_buf_[hdr_len_++] = data[i++];
                if (hdr_len_ < 10) return;
                frame_len_ = 0;
                for (int s = 0; s < 8; ++s) frame_len_ = (frame_len_ << 8) | hdr_buf_[2 + s];
                frame_stage_ = frame_masked_ ? FrameStage::Mask : FrameStage::Payload;
                break;
            }
            case FrameStage::Mask: {
                const size_t mask_off = (hdr_buf_[1] & 0x7F) < 126 ? 2 :
                                        (hdr_buf_[1] & 0x7F) == 126 ? 4 : 10;
                while (hdr_len_ < mask_off + 4 && i < len) hdr_buf_[hdr_len_++] = data[i++];
                if (hdr_len_ < mask_off + 4) return;
                for (int k = 0; k < 4; ++k) frame_mask_[k] = hdr_buf_[mask_off + k];
                frame_stage_ = FrameStage::Payload;
                if (frame_len_ > payload_cap_) {
                    // Too big for our assembly buffer — close with
                    // "Message Too Big" (status 1009).
                    close_with_status(conn, 1009);
                    return;
                }
                payload_filled_ = 0;
                break;
            }
            case FrameStage::Payload: {
                const size_t need = static_cast<size_t>(frame_len_) - payload_filled_;
                const size_t take = (len - i < need) ? (len - i) : need;
                if (payload_buf_ && take > 0) {
                    if (frame_masked_) {
                        for (size_t k = 0; k < take; ++k) {
                            payload_buf_[payload_filled_ + k] =
                                data[i + k] ^ frame_mask_[(payload_filled_ + k) & 3u];
                        }
                    } else {
                        std::memcpy(payload_buf_ + payload_filled_, data + i, take);
                    }
                }
                payload_filled_ += take;
                i += take;
                if (payload_filled_ < frame_len_) return;
                // Frame complete. Dispatch.
                if (frame_opcode_ == 0x1 && frame_fin_) {
                    // Text frame.
                    if (handler_) {
                        handler_->on_ws_text(ws_conn_,
                                             payload_buf_,
                                             static_cast<size_t>(frame_len_));
                    }
                } else if (frame_opcode_ == 0x8) {
                    // Close frame — peer wants to close. Reply with our
                    // close, then TCP-close.
                    close_with_status(conn, 1000);
                }
                // Else: binary / ping / pong — silently drop.
                hdr_len_ = 0;
                frame_stage_ = FrameStage::Header;
                break;
            }
        }
    }
}

void WebSocketServer::close_with_status(TcpConnection& conn, uint16_t status) noexcept {
    if (state_ == State::Closing) return;
    state_ = State::Closing;
    // Send a close frame (opcode 8) carrying a 2-byte status.
    uint8_t close_frame[4];
    close_frame[0] = 0x88;  // FIN | close
    close_frame[1] = 0x02;  // length 2, unmasked
    close_frame[2] = static_cast<uint8_t>(status >> 8);
    close_frame[3] = static_cast<uint8_t>(status);
    (void)conn.send(close_frame, sizeof(close_frame));
    conn.close();
}

} // namespace kernel::net
