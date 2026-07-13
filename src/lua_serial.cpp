#include "lua_serial.h"

#include <HardwareSerial.h>
#include <driver/gpio.h>

extern "C" {
    #include "lua.h"
    #include "lualib.h"
    #include "lauxlib.h"
}

// ============================================================================
// Type-check macros (same pattern as lua_lcd / lua_espnow)
// ============================================================================
#define LUA_CHECK_INT32(L, n)   ((int32_t)luaL_checknumber(L, n))
#define LUA_OPT_INT32(L, n, d)  ((int32_t)luaL_optnumber(L, n, d))

// ============================================================================
// Internal state — exactly one active UART at a time
// ============================================================================
static HardwareSerial *s_uart = nullptr;   // &Serial1 or &Serial2
static int      s_port = 0;                // 0 = none
static int      s_rx_pin = -1;
static int      s_tx_pin = -1;
static uint32_t s_baud = 0;
static char     s_config_str[4] = "8N1";

// read_until() accumulator: bytes drained from the UART while hunting for a
// delimiter. available()/read()/peek()/clear() all treat it as the front of
// the RX stream so no data is ever lost or reordered.
static String s_acc;

static HardwareSerial *port_object(int port) {
    switch (port) {
        case 1: return &Serial1;
        case 2: return &Serial2;
        default: return nullptr;
    }
}

// Return a pin to a clean hi-Z GPIO state so another module can claim it.
static void release_pin(int pin) {
    if (pin >= 0) gpio_reset_pin((gpio_num_t)pin);
}

// ============================================================================
// C API — Lifecycle
// ============================================================================

bool serial_parse_config_c(const char *spec, uint32_t *out_config) {
    if (spec == nullptr || out_config == nullptr) return false;
    if (strlen(spec) != 3) return false;

    uint32_t data;
    switch (spec[0]) {
        case '5': data = 0x0; break;
        case '6': data = 0x4; break;
        case '7': data = 0x8; break;
        case '8': data = 0xc; break;
        default: return false;
    }
    uint32_t parity;
    switch (toupper(spec[1])) {
        case 'N': parity = 0x0; break;
        case 'E': parity = 0x2; break;
        case 'O': parity = 0x3; break;
        default: return false;
    }
    uint32_t stop;
    switch (spec[2]) {
        case '1': stop = 0x10; break;
        case '2': stop = 0x30; break;
        default: return false;
    }
    // Same composition the Arduino core uses for its SERIAL_xxx constants
    // (SERIAL_8N1 == 0x800001c == 0x8000000 | 0x10 | 0x0<<?; verified below):
    // constants are 0x8000000 | data | parity | stop.
    *out_config = 0x8000000 | data | parity | stop;
    return true;
}

bool serial_init_c(int port, uint32_t baud, int rx_pin, int tx_pin,
                   uint32_t config, int rx_buf, int tx_buf, bool invert) {
    if (port < 1 || port > LUA_SERIAL_MAX_PORT) {
        Serial.printf("[SERIAL] init: invalid port %d (valid: 1-%d)\n",
                      port, LUA_SERIAL_MAX_PORT);
        return false;
    }
    if (rx_pin < 0 && tx_pin < 0) {
        Serial.println("[SERIAL] init: need at least one of rx/tx pin");
        return false;
    }
    if (baud == 0) {
        Serial.println("[SERIAL] init: invalid baud 0");
        return false;
    }

    // Single-active rule: opening a port (same or different) closes the
    // previous one first, releasing its pins.
    if (s_uart != nullptr) serial_deinit_c();

    HardwareSerial *uart = port_object(port);
    if (uart == nullptr) return false;

    if (rx_buf > 0) uart->setRxBufferSize(rx_buf);
    uart->setTxBufferSize(tx_buf > 0 ? tx_buf : 0);
    uart->begin(baud, config, rx_pin, tx_pin, invert);
    if (!(*uart)) {
        Serial.printf("[SERIAL] init: UART%d begin failed\n", port);
        uart->end();
        return false;
    }

    s_uart = uart;
    s_port = port;
    s_rx_pin = rx_pin;
    s_tx_pin = tx_pin;
    s_baud = baud;
    s_acc = "";

    Serial.printf("[SERIAL] UART%d open: %lu baud, rx=%d tx=%d\n",
                  port, (unsigned long)baud, rx_pin, tx_pin);
    return true;
}

bool serial_deinit_c(void) {
    if (s_uart == nullptr) return false;

    s_uart->flush();   // don't cut off in-flight TX
    s_uart->end();
    release_pin(s_rx_pin);
    release_pin(s_tx_pin);

    Serial.printf("[SERIAL] UART%d closed, pins %d/%d released\n",
                  s_port, s_rx_pin, s_tx_pin);

    s_uart = nullptr;
    s_port = 0;
    s_rx_pin = -1;
    s_tx_pin = -1;
    s_baud = 0;
    s_acc = "";
    return true;
}

bool serial_is_initialized_c(void) { return s_uart != nullptr; }
int  serial_get_port_c(void)       { return s_port; }

bool serial_set_pins_c(int rx_pin, int tx_pin) {
    if (s_uart == nullptr) return false;
    if (rx_pin < 0 && tx_pin < 0) return false;

    s_uart->flush();
    int old_rx = s_rx_pin, old_tx = s_tx_pin;
    if (!s_uart->setPins(rx_pin, tx_pin)) {
        Serial.printf("[SERIAL] set_pins(%d, %d) failed\n", rx_pin, tx_pin);
        return false;
    }
    // The core re-attaches the UART signals; put the vacated pins back to hi-Z.
    if (old_rx >= 0 && old_rx != rx_pin && old_rx != tx_pin) release_pin(old_rx);
    if (old_tx >= 0 && old_tx != rx_pin && old_tx != tx_pin) release_pin(old_tx);
    s_rx_pin = rx_pin;
    s_tx_pin = tx_pin;
    return true;
}

bool serial_set_baud_c(uint32_t baud) {
    if (s_uart == nullptr || baud == 0) return false;
    s_uart->updateBaudRate(baud);
    s_baud = baud;
    return true;
}

// ============================================================================
// C API — I/O
// ============================================================================

size_t serial_write_c(const uint8_t *data, size_t len) {
    if (s_uart == nullptr || data == nullptr || len == 0) return 0;
    return s_uart->write(data, len);
}

size_t serial_read_c(uint8_t *buf, size_t max_len) {
    if (s_uart == nullptr || buf == nullptr || max_len == 0) return 0;
    size_t n = 0;

    // Serve the read_until accumulator first (it's the oldest data).
    if (s_acc.length() > 0) {
        n = min((size_t)s_acc.length(), max_len);
        memcpy(buf, s_acc.c_str(), n);
        s_acc = s_acc.substring(n);
        if (n == max_len) return n;
    }

    int avail = s_uart->available();
    if (avail > 0) {
        size_t want = min((size_t)avail, max_len - n);
        n += s_uart->readBytes(buf + n, want);
    }
    return n;
}

int serial_available_c(void) {
    if (s_uart == nullptr) return 0;
    return (int)s_acc.length() + s_uart->available();
}

int serial_available_for_write_c(void) {
    if (s_uart == nullptr) return 0;
    return s_uart->availableForWrite();
}

int serial_peek_c(void) {
    if (s_uart == nullptr) return -1;
    if (s_acc.length() > 0) return (uint8_t)s_acc[0];
    return s_uart->peek();
}

void serial_flush_c(void) {
    if (s_uart != nullptr) s_uart->flush();
}

void serial_clear_c(void) {
    if (s_uart == nullptr) return;
    s_acc = "";
    while (s_uart->available() > 0) s_uart->read();
}

// ============================================================================
// C API — Host integration
// ============================================================================

void serial_cleanup_c(void) {
    if (s_uart != nullptr) {
        Serial.printf("[SERIAL] cleanup: closing UART%d left open by script\n", s_port);
        serial_deinit_c();
    }
}

// ============================================================================
// Lua Wrappers
// ============================================================================

// serial.init(port, baud, rx, tx, [opts]) -> bool
// opts: {config="8N1", rxbuf=2048, txbuf=0, invert=false}
static int l_init(lua_State *L) {
    int port      = LUA_CHECK_INT32(L, 1);
    uint32_t baud = (uint32_t)luaL_checknumber(L, 2);
    int rx        = LUA_CHECK_INT32(L, 3);
    int tx        = LUA_CHECK_INT32(L, 4);

    uint32_t config = SERIAL_8N1;
    int rx_buf = LUA_SERIAL_DEFAULT_RX_BUF;
    int tx_buf = LUA_SERIAL_DEFAULT_TX_BUF;
    bool invert = false;

    if (lua_istable(L, 5)) {
        lua_getfield(L, 5, "config");
        if (lua_isstring(L, -1)) {
            if (!serial_parse_config_c(lua_tostring(L, -1), &config)) {
                lua_pop(L, 1);
                return luaL_error(L, "serial.init: bad config string (expect e.g. \"8N1\")");
            }
        }
        lua_pop(L, 1);

        lua_getfield(L, 5, "rxbuf");
        if (lua_isnumber(L, -1)) rx_buf = (int)lua_tonumber(L, -1);
        lua_pop(L, 1);

        lua_getfield(L, 5, "txbuf");
        if (lua_isnumber(L, -1)) tx_buf = (int)lua_tonumber(L, -1);
        lua_pop(L, 1);

        lua_getfield(L, 5, "invert");
        invert = lua_toboolean(L, -1);
        lua_pop(L, 1);
    }

    bool ok = serial_init_c(port, baud, rx, tx, config, rx_buf, tx_buf, invert);
    if (ok) {
        // remember the config string for get_config()
        if (lua_istable(L, 5)) {
            lua_getfield(L, 5, "config");
            if (lua_isstring(L, -1)) {
                strncpy(s_config_str, lua_tostring(L, -1), 3);
                s_config_str[3] = '\0';
            } else {
                strcpy(s_config_str, "8N1");
            }
            lua_pop(L, 1);
        } else {
            strcpy(s_config_str, "8N1");
        }
    }
    lua_pushboolean(L, ok);
    return 1;
}

static int l_deinit(lua_State *L) {
    lua_pushboolean(L, serial_deinit_c());
    return 1;
}

static int l_is_initialized(lua_State *L) {
    lua_pushboolean(L, serial_is_initialized_c());
    return 1;
}

static int l_get_port(lua_State *L) {
    if (s_port == 0) lua_pushnil(L);
    else lua_pushinteger(L, s_port);
    return 1;
}

static int l_set_pins(lua_State *L) {
    int rx = LUA_CHECK_INT32(L, 1);
    int tx = LUA_CHECK_INT32(L, 2);
    lua_pushboolean(L, serial_set_pins_c(rx, tx));
    return 1;
}

static int l_set_baud(lua_State *L) {
    uint32_t baud = (uint32_t)luaL_checknumber(L, 1);
    lua_pushboolean(L, serial_set_baud_c(baud));
    return 1;
}

static int l_get_config(lua_State *L) {
    if (s_uart == nullptr) {
        lua_pushnil(L);
        return 1;
    }
    lua_createtable(L, 0, 5);
    lua_pushinteger(L, s_port);      lua_setfield(L, -2, "port");
    lua_pushinteger(L, s_baud);      lua_setfield(L, -2, "baud");
    lua_pushinteger(L, s_rx_pin);    lua_setfield(L, -2, "rx");
    lua_pushinteger(L, s_tx_pin);    lua_setfield(L, -2, "tx");
    lua_pushstring(L, s_config_str); lua_setfield(L, -2, "config");
    return 1;
}

// serial.write(data) -> bytes written. Binary-safe (embedded NULs fine).
static int l_write(lua_State *L) {
    size_t len = 0;
    const char *data = luaL_checklstring(L, 1, &len);
    lua_pushinteger(L, (lua_Integer)serial_write_c((const uint8_t *)data, len));
    return 1;
}

// serial.print(value) -> bytes written (numbers/booleans coerced like tostring)
static int l_print(lua_State *L) {
    size_t len = 0;
    const char *s = luaL_tolstring(L, 1, &len);   // pushes the string
    size_t n = serial_write_c((const uint8_t *)s, len);
    lua_pop(L, 1);
    lua_pushinteger(L, (lua_Integer)n);
    return 1;
}

// serial.println([value]) -> bytes written (appends "\r\n", Arduino-style)
static int l_println(lua_State *L) {
    size_t n = 0;
    if (lua_gettop(L) >= 1 && !lua_isnil(L, 1)) {
        size_t len = 0;
        const char *s = luaL_tolstring(L, 1, &len);
        n = serial_write_c((const uint8_t *)s, len);
        lua_pop(L, 1);
    }
    n += serial_write_c((const uint8_t *)"\r\n", 2);
    lua_pushinteger(L, (lua_Integer)n);
    return 1;
}

// serial.read([n]) -> binary string ("" if nothing buffered). Non-blocking.
static int l_read(lua_State *L) {
    int avail = serial_available_c();
    int want = LUA_OPT_INT32(L, 1, avail);
    if (want > avail) want = avail;
    if (want <= 0) {
        lua_pushlstring(L, "", 0);
        return 1;
    }

    uint8_t *buf = (uint8_t *)malloc(want);
    if (buf == nullptr) {
        lua_pushlstring(L, "", 0);
        return 1;
    }
    size_t n = serial_read_c(buf, (size_t)want);
    lua_pushlstring(L, (const char *)buf, n);
    free(buf);
    return 1;
}

// serial.read_until(delim, [max]) -> string (without delim) or nil.
// Non-blocking: drains available bytes into an internal accumulator and
// returns a complete chunk only once the delimiter arrives. If the
// accumulator exceeds max (default 512) without a delimiter, the whole
// accumulated blob is returned so binary noise can't grow it unbounded.
static int l_read_until(lua_State *L) {
    size_t dlen = 0;
    const char *dstr = luaL_checklstring(L, 1, &dlen);
    if (dlen != 1) return luaL_error(L, "serial.read_until: delim must be 1 byte");
    char delim = dstr[0];
    int max = LUA_OPT_INT32(L, 2, LUA_SERIAL_DEFAULT_LINE_MAX);
    if (max < 1) max = 1;

    if (s_uart == nullptr) {
        lua_pushnil(L);
        return 1;
    }

    // Drain what's available into the accumulator, watching for the delimiter.
    while (s_uart->available() > 0) {
        int c = s_uart->read();
        if (c < 0) break;
        if ((char)c == delim) {
            lua_pushlstring(L, s_acc.c_str(), s_acc.length());
            s_acc = "";
            return 1;
        }
        s_acc += (char)c;
        if ((int)s_acc.length() >= max) {
            lua_pushlstring(L, s_acc.c_str(), s_acc.length());
            s_acc = "";
            return 1;
        }
    }
    lua_pushnil(L);   // no complete chunk yet; bytes stay accumulated
    return 1;
}

static int l_available(lua_State *L) {
    lua_pushinteger(L, serial_available_c());
    return 1;
}

static int l_available_for_write(lua_State *L) {
    lua_pushinteger(L, serial_available_for_write_c());
    return 1;
}

static int l_peek(lua_State *L) {
    lua_pushinteger(L, serial_peek_c());
    return 1;
}

static int l_flush(lua_State *L) {
    serial_flush_c();
    return 0;
}

static int l_clear(lua_State *L) {
    serial_clear_c();
    return 0;
}

// ============================================================================
// Module registration
// ============================================================================

static const luaL_Reg serial_functions[] = {
    // Lifecycle
    {"init",                l_init},
    {"deinit",              l_deinit},
    {"is_initialized",      l_is_initialized},
    {"get_port",            l_get_port},
    {"set_pins",            l_set_pins},
    {"set_baud",            l_set_baud},
    {"get_config",          l_get_config},

    // Write
    {"write",               l_write},
    {"print",               l_print},
    {"println",             l_println},
    {"flush",               l_flush},
    {"available_for_write", l_available_for_write},

    // Read
    {"available",           l_available},
    {"read",                l_read},
    {"read_until",          l_read_until},
    {"peek",                l_peek},
    {"clear",               l_clear},

    {NULL, NULL}
};

int luaopen_serial(lua_State *L, const char *module_name) {
    luaL_newlib(L, serial_functions);

    lua_pushinteger(L, LUA_SERIAL_MAX_PORT);
    lua_setfield(L, -2, "MAX_PORTS");

    lua_setglobal(L, module_name);
    return 0;
}
