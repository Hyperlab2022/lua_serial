#ifndef LUA_SERIAL_H
#define LUA_SERIAL_H

#include <Arduino.h>

// Forward declaration for Lua
struct lua_State;

// ============================================================================
// lua_serial — single-active-UART Lua module
//
// Unlike most modules, serial is NEVER initialized from C at boot. A Lua
// script selects and opens ONE hardware UART (port 1 or 2; UART0 is the
// console) with pins chosen at runtime:
//
//     serial.init(1, 115200, rx, tx, {config="8N1", rxbuf=2048})
//
// All other calls are id-free and act on that active port. serial.deinit()
// (or serial_cleanup_c() from the host's script-exit hook) closes the UART
// and resets both pins to hi-Z GPIO so other modules can claim them.
// ============================================================================

// Exposed ports: 1..LUA_SERIAL_MAX_PORT (UART0 is the log console — never
// exposed to Lua)
#define LUA_SERIAL_MAX_PORT 2

// Default driver ring-buffer sizes (bytes). RX is generous so a burst of
// ESP-NOW packets bridged to a slower UART doesn't drop bytes; both are
// overridable per-open via the Lua opts table.
#define LUA_SERIAL_DEFAULT_RX_BUF 2048
#define LUA_SERIAL_DEFAULT_TX_BUF 0

// Cap for the read_until() line accumulator when no delimiter shows up
// (overridable per call). Prevents unbounded growth on binary streams.
#define LUA_SERIAL_DEFAULT_LINE_MAX 512

// ============================================================================
// C API — Lifecycle
// ============================================================================
// config is an Arduino SERIAL_xxx constant (e.g. SERIAL_8N1); use
// serial_parse_config_c to build it from a "8N1"-style string.
bool serial_init_c(int port, uint32_t baud, int rx_pin, int tx_pin,
                   uint32_t config = SERIAL_8N1,
                   int rx_buf = LUA_SERIAL_DEFAULT_RX_BUF,
                   int tx_buf = LUA_SERIAL_DEFAULT_TX_BUF,
                   bool invert = false);
bool serial_deinit_c(void);              // close + release pins (false if not open)
bool serial_is_initialized_c(void);
int  serial_get_port_c(void);            // 0 = none active
bool serial_set_pins_c(int rx_pin, int tx_pin);   // live pin move, old pins released
bool serial_set_baud_c(uint32_t baud);

// "8N1" / "7E2" ... -> SERIAL_xxx constant. Returns false on a malformed spec.
bool serial_parse_config_c(const char *spec, uint32_t *out_config);

// ============================================================================
// C API — I/O (binary-safe)
// ============================================================================
size_t serial_write_c(const uint8_t *data, size_t len);
size_t serial_read_c(uint8_t *buf, size_t max_len);  // non-blocking drain
int    serial_available_c(void);
int    serial_available_for_write_c(void);
int    serial_peek_c(void);                          // -1 if empty
void   serial_flush_c(void);                         // block until TX drained
void   serial_clear_c(void);                         // discard buffered RX

// ============================================================================
// C API — Host integration
// ============================================================================
// Script-exit hook: force-close the active UART and release its pins.
// Call from cleanupCustomHardware() so a crashed script can't hold pins.
void serial_cleanup_c(void);

// ============================================================================
// Lua Module Registration
// ============================================================================
int luaopen_serial(lua_State *L, const char *module_name);

#endif // LUA_SERIAL_H
