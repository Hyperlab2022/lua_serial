#ifndef RMT_SERIAL_H
#define RMT_SERIAL_H

// ============================================================================
// RmtSerial — a "software" UART for ESP32-S3 built on the RMT peripheral.
//
// The RMT hardware times every bit, so this is not bit-banged: TX never
// busy-waits or disables interrupts, and RX gets hardware timestamps for each
// edge. It is written on the ESP-IDF 5 RMT driver (the same driver Arduino's
// rmtInit()/NeoPixel use), so it shares the RMT peripheral cleanly with them:
// the driver hands out channels and shares the RMT interrupt.
//
// How it works
//   TX  Each write is cut into chunks small enough to sit entirely in one
//       48-word RMT memory block (9 bytes at 8N1). Every chunk is its own RMT
//       transaction, so the driver never refills memory mid-chunk: a late
//       interrupt (BLE, a flash write) can only lengthen the idle gap between
//       chunks, never corrupt a byte. Chunks are queued; write() returns as
//       soon as they are queued and only blocks when the queue is full.
//   RX  Continuous receive in ping-pong mode (two RMT blocks when free, one
//       otherwise). The driver's callback hands over symbol batches, which are
//       decoded in the ISR by sampling each bit at its centre, re-syncing on
//       every start bit, into a lock-free ring buffer. When the line has been
//       idle for ~32 bit times the RMT ends the frame; the callback re-arms the
//       receiver straight away.
//
// Limits
//   - Frame formats: 5-8 data bits, parity N/E/O, 1 or 2 stop bits.
//   - Baud 300 .. 2,000,000 (designed for 1200 .. 460800).
//   - Uses 1 RMT TX channel and 1 RMT RX channel (+ 1 borrowed RX block).
//   - The RMT ISR is not IRAM-safe in the Arduino build, so it cannot run
//     while flash is being written/erased (BLE file upload, storage.set, OTA).
//     TX simply pauses; an RX burst arriving during a long erase can lose
//     bytes. Put a checksum in your frames.
//
// Approach inspired by qqqlab/ESP32-Serial-RMT-UART (MIT), reimplemented on
// the public IDF RMT driver API instead of raw RMT registers.
// ============================================================================

#include <Arduino.h>
#include <Stream.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Max UART characters per TX chunk (5N1 packs 11 into one memory block).
#define RMT_SERIAL_MAX_CHUNK 11
// RX ping-pong half size when two RMT blocks are available.
#define RMT_SERIAL_RX_PP_MAX 48

class RmtSerial : public Stream {
public:
    RmtSerial() = default;
    ~RmtSerial() { end(); }
    RmtSerial(const RmtSerial &) = delete;
    RmtSerial &operator=(const RmtSerial &) = delete;

    // config: an Arduino SERIAL_xxx constant (e.g. SERIAL_8N1).
    // rx_pin / tx_pin: -1 for an unused direction (not both).
    // rx_buf: RX ring bytes. tx_buf: bytes that can be queued before write()
    // blocks (rounded to whole chunks).
    bool begin(uint32_t baud, uint32_t config = SERIAL_8N1,
               int rx_pin = -1, int tx_pin = -1, bool invert = false,
               size_t rx_buf = 2048, size_t tx_buf = 256);
    void end();

    // Live changes: TX is drained, the RMT channels are rebuilt, buffered RX
    // bytes are kept. On failure the old setting is restored.
    bool setPins(int rx_pin, int tx_pin);
    bool updateBaudRate(uint32_t baud);

    bool isOpen() const { return m_open; }
    explicit operator bool() const { return m_open; }
    uint32_t baudRate() const { return m_baud; }

    // Stream / Print
    int available() override;
    int read() override;
    int peek() override;
    size_t readBytes(char *buffer, size_t length) override;   // non-blocking
    size_t readBytes(uint8_t *buffer, size_t length) override {
        return readBytes((char *)buffer, length);
    }
    size_t write(uint8_t b) override { return write(&b, 1); }
    size_t write(const uint8_t *data, size_t len) override;
    using Print::write;
    int availableForWrite() override;
    void flush() override;                 // block until TX is on the wire
    void clearRx();                        // discard buffered RX

    // Diagnostics (cumulative since begin()).
    uint32_t rxOverflows() const    { return __atomic_load_n(&m_rx_overflows, __ATOMIC_RELAXED); }
    uint32_t rxFrameErrors() const  { return __atomic_load_n(&m_rx_frame_errors, __ATOMIC_RELAXED); }
    uint32_t rxParityErrors() const { return __atomic_load_n(&m_rx_parity_errors, __ATOMIC_RELAXED); }

private:
    bool parseConfig(uint32_t config);
    bool startChannels();
    void stopChannels();
    bool startTx();
    bool startRx();
    void stopTx();
    void stopRx();
    void armRx();

    void encodeChar(uint8_t c, rmt_symbol_word_t *out) const;
    void decodeSymbols(const rmt_symbol_word_t *sym, size_t n);
    void decodeRun(uint32_t level, uint32_t duration);
    void decodeSample(uint32_t level);
    void pushRx(uint8_t b);

    static size_t txEncodeCb(const void *data, size_t data_size,
                             size_t symbols_written, size_t symbols_free,
                             rmt_symbol_word_t *symbols, bool *done, void *arg);
    static bool txDoneCb(rmt_channel_handle_t chan,
                         const rmt_tx_done_event_data_t *edata, void *arg);
    static bool rxDoneCb(rmt_channel_handle_t chan,
                         const rmt_rx_done_event_data_t *edata, void *arg);

    // --- configuration
    bool     m_open = false;
    uint32_t m_baud = 0;
    int      m_rx_pin = -1;
    int      m_tx_pin = -1;
    bool     m_invert = false;
    uint8_t  m_data_bits = 8;
    uint8_t  m_parity = 0;       // 0 none, 1 even, 2 odd
    uint8_t  m_stop_bits = 1;
    uint8_t  m_frame_bits = 10;  // start + data + parity + stop

    // --- TX
    rmt_channel_handle_t m_tx_chan = nullptr;
    rmt_encoder_handle_t m_tx_enc = nullptr;
    SemaphoreHandle_t    m_tx_sem = nullptr;  // counts free chunk slots
    uint8_t  *m_tx_slots = nullptr;           // m_tx_depth * RMT_SERIAL_MAX_CHUNK
    size_t    m_tx_depth = 0;
    size_t    m_tx_buf_req = 256;
    size_t    m_tx_slot = 0;                  // next slot to fill
    uint32_t  m_tx_bit_ticks = 0;
    uint8_t   m_tx_words_per_char = 5;
    uint8_t   m_tx_chars_per_chunk = 9;
    uint32_t  m_tx_chunk_ms = 1;              // wire time of one full chunk
    uint32_t  m_tx_timeout_ms = 1000;         // max wait for one free slot

    // --- RX
    rmt_channel_handle_t m_rx_chan = nullptr;
    rmt_receive_config_t m_rx_cfg = {};
    rmt_symbol_word_t    m_rx_sym[RMT_SERIAL_RX_PP_MAX] = {};
    size_t   m_rx_pp = 0;                     // ping-pong half, in symbols
    uint32_t m_rx_bit_q8 = 0;                 // RMT ticks per bit, x256
    volatile bool m_rx_armed = false;
    volatile bool m_rx_closing = false;

    uint8_t *m_rx_buf = nullptr;              // ring, m_rx_size slots
    size_t   m_rx_size = 0;
    size_t   m_rx_head = 0;                   // written by ISR
    size_t   m_rx_tail = 0;                   // written by reader

    // decoder (ISR only)
    bool     m_dec_in_char = false;
    uint32_t m_dec_prev_level = 1;
    uint32_t m_dec_t = 0;                     // ticks since start edge
    uint8_t  m_dec_bit = 0;                   // next bit index to sample
    uint8_t  m_dec_value = 0;
    uint8_t  m_dec_ones = 0;
    bool     m_dec_parity_ok = true;

    // written only by the RX ISR, read with atomic loads
    uint32_t m_rx_overflows = 0;
    uint32_t m_rx_frame_errors = 0;
    uint32_t m_rx_parity_errors = 0;
};

#endif // RMT_SERIAL_H
