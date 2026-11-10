#include "rmt_serial.h"

#include <driver/gpio.h>
#include <esp_clk_tree.h>
#include <esp_heap_caps.h>
#include <soc/soc_caps.h>

// ============================================================================
// Tuning
// ============================================================================
static const uint32_t MIN_BAUD = 300;
static const uint32_t MAX_BAUD = 2000000;
// RX frame ends after the line has been idle this many bit times. Long enough
// that a normal burst stays in one RMT frame (no re-arm gaps), short enough
// that the tail of a burst is delivered quickly (~0.3 ms at 115200).
static const uint32_t RX_IDLE_BITS = 32;
static const uint32_t RMT_MAX_TICKS = 32767;     // 15-bit duration / idle field
static const uint32_t RMT_MAX_FILTER_NS = 3000;  // filter reg is 8 bits of 80 MHz
static const size_t   TX_MIN_DEPTH = 4;
static const size_t   TX_MAX_DEPTH = 128;
static const size_t   RX_MIN_BUF = 64;

static uint32_t rmt_src_hz() {
    uint32_t hz = 0;
    if (esp_clk_tree_src_get_freq_hz((soc_module_clk_t)RMT_CLK_SRC_DEFAULT,
                                     ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &hz) != ESP_OK ||
        hz == 0) {
        hz = 80000000;
    }
    return hz;
}

// Smallest channel divider (1..256) that keeps `ticks` within the 15-bit RMT
// range for a span of `bits` bit times.
static uint32_t pick_div(uint32_t src_hz, uint32_t baud, uint32_t bits) {
    uint64_t denom = (uint64_t)baud * RMT_MAX_TICKS;
    uint64_t div = ((uint64_t)src_hz * bits + denom - 1) / denom;
    if (div < 1) div = 1;
    if (div > 256) div = 256;
    return (uint32_t)div;
}

static bool pins_ok(int rx_pin, int tx_pin) {
    if (rx_pin < 0 && tx_pin < 0) return false;
    if (rx_pin >= 0 && rx_pin == tx_pin) return false;   // no half-duplex
    if (tx_pin >= 0 && !GPIO_IS_VALID_OUTPUT_GPIO(tx_pin)) return false;
    if (rx_pin >= 0 && !GPIO_IS_VALID_GPIO(rx_pin)) return false;
    return true;
}

// ============================================================================
// Lifecycle
// ============================================================================

bool RmtSerial::parseConfig(uint32_t config) {
    // Arduino SERIAL_xxx layout: bits 0-1 parity (0 none, 2 even, 3 odd),
    // bits 2-3 data bits - 5, bits 4-5 stop (1 = one, 3 = two, 2 = 1.5).
    uint32_t parity = config & 0x3;
    uint32_t data = (config >> 2) & 0x3;
    uint32_t stop = (config >> 4) & 0x3;
    if (parity == 1) return false;
    if (stop != 1 && stop != 3) return false;   // 1.5 stop bits not supported
    m_data_bits = 5 + data;
    m_parity = parity == 0 ? 0 : (parity == 2 ? 1 : 2);
    m_stop_bits = stop == 1 ? 1 : 2;
    m_frame_bits = 1 + m_data_bits + (m_parity ? 1 : 0) + m_stop_bits;
    return true;
}

bool RmtSerial::begin(uint32_t baud, uint32_t config, int rx_pin, int tx_pin,
                      bool invert, size_t rx_buf, size_t tx_buf) {
    end();

    if (baud < MIN_BAUD || baud > MAX_BAUD) {
        log_e("RmtSerial: baud %lu out of range %lu..%lu", (unsigned long)baud,
              (unsigned long)MIN_BAUD, (unsigned long)MAX_BAUD);
        return false;
    }
    if (!pins_ok(rx_pin, tx_pin)) {
        log_e("RmtSerial: bad pins rx=%d tx=%d", rx_pin, tx_pin);
        return false;
    }
    if (!parseConfig(config)) {
        log_e("RmtSerial: unsupported config 0x%lx", (unsigned long)config);
        return false;
    }

    m_baud = baud;
    m_rx_pin = rx_pin;
    m_tx_pin = tx_pin;
    m_invert = invert;
    m_rx_size = (rx_buf < RX_MIN_BUF ? RX_MIN_BUF : rx_buf) + 1;   // ring keeps one slot empty
    m_tx_buf_req = tx_buf > 0 ? tx_buf : 256;
    m_rx_overflows = 0;
    m_rx_frame_errors = 0;
    m_rx_parity_errors = 0;

    if (!startChannels()) {
        end();
        return false;
    }
    m_open = true;
    return true;
}

void RmtSerial::end() {
    stopChannels();
    free(m_rx_buf);
    m_rx_buf = nullptr;
    m_rx_head = 0;
    m_rx_tail = 0;
    m_open = false;
    m_rx_pin = -1;
    m_tx_pin = -1;
}

bool RmtSerial::setPins(int rx_pin, int tx_pin) {
    if (!m_open || !pins_ok(rx_pin, tx_pin)) return false;
    int old_rx = m_rx_pin, old_tx = m_tx_pin;
    stopChannels();
    m_rx_pin = rx_pin;
    m_tx_pin = tx_pin;
    if (startChannels()) return true;
    m_rx_pin = old_rx;
    m_tx_pin = old_tx;
    if (!startChannels()) m_open = false;
    return false;
}

bool RmtSerial::updateBaudRate(uint32_t baud) {
    if (!m_open || baud < MIN_BAUD || baud > MAX_BAUD) return false;
    if (baud == m_baud) return true;
    uint32_t old = m_baud;
    stopChannels();
    m_baud = baud;
    if (startChannels()) return true;
    m_baud = old;
    if (!startChannels()) m_open = false;
    return false;
}

bool RmtSerial::startChannels() {
    if (m_tx_pin >= 0 && !startTx()) {
        stopChannels();
        return false;
    }
    if (m_rx_pin >= 0 && !startRx()) {
        stopChannels();
        return false;
    }
    return true;
}

void RmtSerial::stopChannels() {
    stopRx();
    stopTx();
}

// ============================================================================
// TX
// ============================================================================

bool RmtSerial::startTx() {
    const uint32_t src = rmt_src_hz();
    const uint32_t div = pick_div(src, m_baud, 1);
    const uint32_t res = src / div;
    m_tx_bit_ticks = (res + m_baud / 2) / m_baud;
    if (m_tx_bit_ticks < 2 || m_tx_bit_ticks > RMT_MAX_TICKS) {
        log_e("RmtSerial: TX bit time %lu ticks unusable", (unsigned long)m_tx_bit_ticks);
        return false;
    }

    // One RMT word holds two bit halves; an odd frame is padded by splitting
    // the last stop bit. Leave one word of the block for the driver's EOF mark.
    m_tx_words_per_char = (m_frame_bits + 1) / 2;
    m_tx_chars_per_chunk = (SOC_RMT_MEM_WORDS_PER_CHANNEL - 1) / m_tx_words_per_char;
    if (m_tx_chars_per_chunk > RMT_SERIAL_MAX_CHUNK) m_tx_chars_per_chunk = RMT_SERIAL_MAX_CHUNK;

    size_t depth = (m_tx_buf_req + m_tx_chars_per_chunk - 1) / m_tx_chars_per_chunk;
    if (depth < TX_MIN_DEPTH) depth = TX_MIN_DEPTH;
    if (depth > TX_MAX_DEPTH) depth = TX_MAX_DEPTH;
    m_tx_depth = depth;

    m_tx_chunk_ms = (uint32_t)((uint64_t)m_tx_chars_per_chunk * m_frame_bits * 1000 / m_baud) + 1;
    // A queued chunk can wait out a flash sector erase (RMT ISR is blocked).
    m_tx_timeout_ms = 1000 + 2 * m_tx_chunk_ms;
    m_tx_slot = 0;

    m_tx_slots = (uint8_t *)malloc(m_tx_depth * RMT_SERIAL_MAX_CHUNK);
    m_tx_sem = xSemaphoreCreateCounting(m_tx_depth, m_tx_depth);
    if (m_tx_slots == nullptr || m_tx_sem == nullptr) {
        log_e("RmtSerial: TX alloc failed");
        stopTx();
        return false;
    }

    rmt_tx_channel_config_t tc = {};
    tc.gpio_num = (gpio_num_t)m_tx_pin;
    tc.clk_src = RMT_CLK_SRC_DEFAULT;
    tc.resolution_hz = res;
    tc.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
    tc.trans_queue_depth = m_tx_depth;
    // The RMT idles LOW before its first transaction. Inverting the output
    // makes that idle read HIGH on the wire (UART idle), so opening the port
    // never glitches the line. Symbol levels are encoded inverted to match.
    tc.flags.invert_out = !m_invert;
    esp_err_t err = rmt_new_tx_channel(&tc, &m_tx_chan);
    if (err != ESP_OK) {
        log_e("RmtSerial: no RMT TX channel (%s)", esp_err_to_name(err));
        m_tx_chan = nullptr;
        stopTx();
        return false;
    }

    rmt_simple_encoder_config_t ec = {};
    ec.callback = txEncodeCb;
    ec.arg = this;
    ec.min_chunk_size = m_tx_words_per_char;
    rmt_tx_event_callbacks_t cbs = {};
    cbs.on_trans_done = txDoneCb;
    if (rmt_new_simple_encoder(&ec, &m_tx_enc) != ESP_OK ||
        rmt_tx_register_event_callbacks(m_tx_chan, &cbs, this) != ESP_OK ||
        rmt_enable(m_tx_chan) != ESP_OK) {
        log_e("RmtSerial: TX setup failed");
        stopTx();
        return false;
    }
    return true;
}

void RmtSerial::stopTx() {
    if (m_tx_chan) {
        flush();
        rmt_disable(m_tx_chan);
        rmt_del_channel(m_tx_chan);
        m_tx_chan = nullptr;
    }
    if (m_tx_enc) {
        rmt_del_encoder(m_tx_enc);
        m_tx_enc = nullptr;
    }
    if (m_tx_sem) {
        vSemaphoreDelete(m_tx_sem);
        m_tx_sem = nullptr;
    }
    free(m_tx_slots);
    m_tx_slots = nullptr;
    m_tx_depth = 0;
}

// Encode one character as m_tx_words_per_char RMT words (one bit per half).
void RmtSerial::encodeChar(uint8_t c, rmt_symbol_word_t *out) const {
    uint32_t bits = 0;   // bit i = wire level of frame bit i (start bit = 0)
    uint32_t n = 1;
    uint32_t ones = 0;
    for (uint32_t k = 0; k < m_data_bits; k++) {
        uint32_t b = (c >> k) & 1u;
        ones += b;
        bits |= b << n++;
    }
    if (m_parity) {
        uint32_t p = (ones & 1u) ^ (m_parity == 2 ? 1u : 0u);   // even / odd
        bits |= p << n++;
    }
    for (uint32_t k = 0; k < m_stop_bits; k++) bits |= 1u << n++;

    uint16_t level[14];
    uint16_t dur[14];
    uint32_t h = 0;
    for (uint32_t i = 0; i < n; i++, h++) {
        level[h] = ((bits >> i) & 1u) ^ 1u;   // output is inverted, see startTx()
        dur[h] = (uint16_t)m_tx_bit_ticks;
    }
    if (h & 1u) {   // odd frame: split the last stop bit into two halves
        dur[h - 1] = (uint16_t)(m_tx_bit_ticks / 2);
        level[h] = level[h - 1];
        dur[h] = (uint16_t)(m_tx_bit_ticks - m_tx_bit_ticks / 2);
        h++;
    }
    for (uint32_t w = 0; w < h / 2; w++) {
        out[w].level0 = level[2 * w];
        out[w].duration0 = dur[2 * w];
        out[w].level1 = level[2 * w + 1];
        out[w].duration1 = dur[2 * w + 1];
    }
}

// Simple-encoder callback; runs in task context (first chunk) or in the RMT
// ISR (chained chunks). Payload = the chunk's raw bytes.
size_t RmtSerial::txEncodeCb(const void *data, size_t data_size,
                             size_t symbols_written, size_t symbols_free,
                             rmt_symbol_word_t *symbols, bool *done, void *arg) {
    const RmtSerial *self = (const RmtSerial *)arg;
    const size_t w = self->m_tx_words_per_char;
    const uint8_t *bytes = (const uint8_t *)data;
    size_t idx = symbols_written / w;
    size_t out = 0;
    while (idx < data_size && symbols_free - out >= w) {
        self->encodeChar(bytes[idx], symbols + out);
        out += w;
        idx++;
    }
    *done = idx >= data_size;
    return out;
}

bool RmtSerial::txDoneCb(rmt_channel_handle_t, const rmt_tx_done_event_data_t *, void *arg) {
    RmtSerial *self = (RmtSerial *)arg;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(self->m_tx_sem, &woken);
    return woken == pdTRUE;
}

size_t RmtSerial::write(const uint8_t *data, size_t len) {
    if (m_tx_chan == nullptr || data == nullptr || len == 0) return 0;

    rmt_transmit_config_t cfg = {};
    cfg.loop_count = 0;
    cfg.flags.eot_level = 0;            // RMT low == wire idle (output inverted)
    cfg.flags.queue_nonblocking = 1;    // our semaphore already guarantees a slot

    size_t sent = 0;
    while (sent < len) {
        if (xSemaphoreTake(m_tx_sem, pdMS_TO_TICKS(m_tx_timeout_ms)) != pdTRUE) {
            log_w("RmtSerial: TX stalled, %u of %u bytes queued", (unsigned)sent, (unsigned)len);
            break;
        }
        size_t n = len - sent;
        if (n > m_tx_chars_per_chunk) n = m_tx_chars_per_chunk;
        // Slots free up in FIFO order, so holding a semaphore count means the
        // slot at m_tx_slot is no longer referenced by the driver.
        uint8_t *slot = m_tx_slots + m_tx_slot * RMT_SERIAL_MAX_CHUNK;
        memcpy(slot, data + sent, n);
        if (rmt_transmit(m_tx_chan, m_tx_enc, slot, n, &cfg) != ESP_OK) {
            xSemaphoreGive(m_tx_sem);
            break;
        }
        m_tx_slot = (m_tx_slot + 1) % m_tx_depth;
        sent += n;
    }
    return sent;
}

int RmtSerial::availableForWrite() {
    if (m_tx_chan == nullptr) return 0;
    return (int)(uxSemaphoreGetCount(m_tx_sem) * m_tx_chars_per_chunk);
}

void RmtSerial::flush() {
    if (m_tx_chan == nullptr) return;
    const uint32_t start = millis();
    const uint32_t limit = m_tx_timeout_ms + (uint32_t)m_tx_depth * m_tx_chunk_ms;
    while (uxSemaphoreGetCount(m_tx_sem) < m_tx_depth) {
        if (millis() - start > limit) {
            log_w("RmtSerial: flush timeout");
            break;
        }
        vTaskDelay(1);
    }
}

// ============================================================================
// RX
// ============================================================================

bool RmtSerial::startRx() {
    if (m_rx_buf == nullptr) {
        m_rx_buf = (uint8_t *)heap_caps_malloc(m_rx_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (m_rx_buf == nullptr) m_rx_buf = (uint8_t *)malloc(m_rx_size);
        if (m_rx_buf == nullptr) {
            log_e("RmtSerial: RX buffer alloc failed (%u)", (unsigned)m_rx_size);
            return false;
        }
        m_rx_head = 0;
        m_rx_tail = 0;
    }

    const uint32_t src = rmt_src_hz();
    const uint32_t div = pick_div(src, m_baud, RX_IDLE_BITS);
    const uint32_t res = src / div;
    m_rx_bit_q8 = (uint32_t)(((uint64_t)res << 8) / m_baud);
    const uint32_t bit_ticks = m_rx_bit_q8 >> 8;
    uint32_t idle_ticks = RX_IDLE_BITS * bit_ticks;
    if (idle_ticks > RMT_MAX_TICKS) idle_ticks = RMT_MAX_TICKS;
    if (bit_ticks < 4 || idle_ticks < (uint32_t)(m_frame_bits + 2) * bit_ticks) {
        log_e("RmtSerial: RX timing unusable at %lu baud", (unsigned long)m_baud);
        return false;
    }

    uint32_t filter_ns = 1000000000UL / m_baud / 8;   // ignore pulses < 1/8 bit
    if (filter_ns > RMT_MAX_FILTER_NS) filter_ns = RMT_MAX_FILTER_NS;
    m_rx_cfg = {};
    m_rx_cfg.signal_range_min_ns = filter_ns;
    m_rx_cfg.signal_range_max_ns = (uint32_t)((uint64_t)idle_ticks * 1000000000ULL / res);
    m_rx_cfg.flags.en_partial_rx = 1;

    rmt_rx_channel_config_t rc = {};
    rc.gpio_num = (gpio_num_t)m_rx_pin;
    rc.clk_src = RMT_CLK_SRC_DEFAULT;
    rc.resolution_hz = res;
    rc.mem_block_symbols = 2 * SOC_RMT_MEM_WORDS_PER_CHANNEL;   // borrow a 2nd block
    rc.flags.invert_in = m_invert;
    esp_err_t err = rmt_new_rx_channel(&rc, &m_rx_chan);
    if (err != ESP_OK) {
        rc.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;    // fall back to one
        err = rmt_new_rx_channel(&rc, &m_rx_chan);
    }
    if (err != ESP_OK) {
        log_e("RmtSerial: no RMT RX channel (%s)", esp_err_to_name(err));
        m_rx_chan = nullptr;
        return false;
    }
    m_rx_pp = rc.mem_block_symbols / 2;

    m_dec_in_char = false;
    m_dec_prev_level = 1;
    m_rx_closing = false;

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = rxDoneCb;
    if (rmt_rx_register_event_callbacks(m_rx_chan, &cbs, this) != ESP_OK ||
        rmt_enable(m_rx_chan) != ESP_OK) {
        log_e("RmtSerial: RX setup failed");
        stopRx();
        return false;
    }
    armRx();
    if (!m_rx_armed) {
        log_e("RmtSerial: RX start failed");
        stopRx();
        return false;
    }
    return true;
}

void RmtSerial::stopRx() {
    if (m_rx_chan == nullptr) return;
    m_rx_closing = true;   // the callback must not re-arm from here on
    rmt_disable(m_rx_chan);
    rmt_del_channel(m_rx_chan);
    m_rx_chan = nullptr;
    m_rx_armed = false;
}

void RmtSerial::armRx() {
    m_rx_armed = rmt_receive(m_rx_chan, m_rx_sym, m_rx_pp * sizeof(rmt_symbol_word_t),
                             &m_rx_cfg) == ESP_OK;
}

// Driver callback (ISR). With a user buffer of exactly one ping-pong half,
// it fires once per half of received symbols, and once more (is_last) when
// the line goes idle and the receiver stops.
bool RmtSerial::rxDoneCb(rmt_channel_handle_t, const rmt_rx_done_event_data_t *edata, void *arg) {
    RmtSerial *self = (RmtSerial *)arg;
    if (edata->flags.is_last) {
        // Re-arm before decoding to keep the dead time minimal. The symbols in
        // edata stay valid: the driver only copies into m_rx_sym again from
        // this same ISR, after we return.
        if (self->m_rx_closing) self->m_rx_armed = false;
        else self->armRx();
    }
    self->decodeSymbols(edata->received_symbols, edata->num_symbols);
    if (edata->flags.is_last && self->m_dec_in_char) {
        // Frame ended mid-character (cannot happen with a proper idle run).
        self->m_dec_in_char = false;
        self->m_rx_frame_errors++;
    }
    return false;
}

void RmtSerial::decodeSymbols(const rmt_symbol_word_t *sym, size_t n) {
    for (size_t i = 0; i < n; i++) {
        decodeRun(sym[i].level0, sym[i].duration0);
        if (sym[i].duration0 == 0) return;   // end marker
        decodeRun(sym[i].level1, sym[i].duration1);
        if (sym[i].duration1 == 0) return;
    }
}

// One run of constant level. duration 0 is the RMT end marker: the line kept
// this level past the idle threshold, i.e. "forever" for decoding purposes.
void RmtSerial::decodeRun(uint32_t level, uint32_t duration) {
    if (!m_dec_in_char) {
        if (level == 0 && m_dec_prev_level != 0) {   // falling edge = start bit
            m_dec_in_char = true;
            m_dec_t = 0;
            m_dec_bit = 0;
            m_dec_value = 0;
            m_dec_ones = 0;
            m_dec_parity_ok = true;
        } else {
            m_dec_prev_level = level;
            return;
        }
    }
    const uint32_t run_end = duration == 0 ? UINT32_MAX : m_dec_t + duration;
    // Sample every bit centre that falls inside this run.
    while (m_dec_in_char) {
        uint32_t centre = ((2u * m_dec_bit + 1u) * m_rx_bit_q8) >> 9;
        if (centre >= run_end) break;
        decodeSample(level);
    }
    if (m_dec_in_char) m_dec_t = run_end;
    m_dec_prev_level = level;
}

void RmtSerial::decodeSample(uint32_t level) {
    const uint8_t b = m_dec_bit;
    if (b == 0) {
        if (level != 0) {   // too short to be a start bit: a glitch
            m_dec_in_char = false;
            return;
        }
    } else if (b <= m_data_bits) {
        if (level) {
            m_dec_value |= (uint8_t)(1u << (b - 1));
            m_dec_ones++;
        }
    } else if (m_parity && b == m_data_bits + 1) {
        uint32_t ones = m_dec_ones + (level ? 1u : 0u);
        m_dec_parity_ok = (m_parity == 1) ? ((ones & 1u) == 0) : ((ones & 1u) == 1);
    } else {   // first stop bit; any further stop bits read as idle
        m_dec_in_char = false;
        if (!level) {
            m_rx_frame_errors++;
        } else if (!m_dec_parity_ok) {
            m_rx_parity_errors++;
        } else {
            pushRx(m_dec_value);
        }
        return;
    }
    m_dec_bit++;
}

void RmtSerial::pushRx(uint8_t b) {
    size_t head = m_rx_head;
    size_t next = head + 1;
    if (next == m_rx_size) next = 0;
    if (next == __atomic_load_n(&m_rx_tail, __ATOMIC_ACQUIRE)) {
        m_rx_overflows++;
        return;
    }
    m_rx_buf[head] = b;
    __atomic_store_n(&m_rx_head, next, __ATOMIC_RELEASE);
}

// ============================================================================
// RX reader side (single consumer)
// ============================================================================

int RmtSerial::available() {
    if (m_rx_buf == nullptr) return 0;
    // Self-heal if a re-arm from the ISR ever failed.
    if (m_rx_chan && !m_rx_armed && !m_rx_closing) armRx();
    size_t head = __atomic_load_n(&m_rx_head, __ATOMIC_ACQUIRE);
    size_t tail = m_rx_tail;
    return (int)((head + m_rx_size - tail) % m_rx_size);
}

int RmtSerial::peek() {
    if (m_rx_buf == nullptr) return -1;
    size_t tail = m_rx_tail;
    if (__atomic_load_n(&m_rx_head, __ATOMIC_ACQUIRE) == tail) return -1;
    return m_rx_buf[tail];
}

int RmtSerial::read() {
    if (m_rx_buf == nullptr) return -1;
    size_t tail = m_rx_tail;
    if (__atomic_load_n(&m_rx_head, __ATOMIC_ACQUIRE) == tail) return -1;
    int b = m_rx_buf[tail];
    __atomic_store_n(&m_rx_tail, (tail + 1) % m_rx_size, __ATOMIC_RELEASE);
    return b;
}

size_t RmtSerial::readBytes(char *buffer, size_t length) {
    if (m_rx_buf == nullptr || buffer == nullptr) return 0;
    size_t head = __atomic_load_n(&m_rx_head, __ATOMIC_ACQUIRE);
    size_t tail = m_rx_tail;
    size_t n = 0;
    while (n < length && tail != head) {
        buffer[n++] = (char)m_rx_buf[tail];
        tail = (tail + 1) % m_rx_size;
    }
    __atomic_store_n(&m_rx_tail, tail, __ATOMIC_RELEASE);
    return n;
}

void RmtSerial::clearRx() {
    if (m_rx_buf == nullptr) return;
    __atomic_store_n(&m_rx_tail, __atomic_load_n(&m_rx_head, __ATOMIC_ACQUIRE), __ATOMIC_RELEASE);
}
