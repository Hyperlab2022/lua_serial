---@meta lua_serial
--- Serial (UART) Lua Type Definitions
--- Single-active-port design: serial.init(port, ...) selects THE port,
--- every other call is id-free and acts on it. Binary-safe read/write.
--- Ports 1-2 are hardware UARTs; port 3 is the RMT software UART.

-- =============================================================================
-- Serial Module
-- =============================================================================

---@class serial
local serial = {}

-- =============================================================================
-- Constants
-- =============================================================================

---Highest selectable port number (UART0 is the console and never exposed)
---@type integer
serial.MAX_PORTS = 3

---Port number of the RMT software UART (any free GPIOs, same API)
---@type integer
serial.SOFT_PORT = 3

-- =============================================================================
-- Types
-- =============================================================================

---Options for serial.init
---@class SerialInitOpts
---@field config string?  Frame format like "8N1", "7E2" (default "8N1")
---@field rxbuf integer?  RX ring buffer bytes (default 2048)
---@field txbuf integer?  TX ring buffer bytes (default 0 = FIFO only, writes may block; port 3: <=0 means 256)
---@field invert boolean? Invert RX/TX logic levels (default false)

---Active-port info returned by serial.get_config
---@class SerialConfig
---@field port integer   Active port number (1, 2, or 3)
---@field baud integer   Current baud rate
---@field rx integer     RX GPIO
---@field tx integer     TX GPIO
---@field config string  Frame format string, e.g. "8N1"
---@field soft boolean   True for the RMT software UART (port 3)

---Soft-port receive error counters returned by serial.get_stats
---@class SerialStats
---@field rx_overflows integer     Bytes dropped because the RX buffer was full
---@field rx_frame_errors integer  Characters dropped for a missing stop bit
---@field rx_parity_errors integer Characters dropped for a parity mismatch

-- =============================================================================
-- Lifecycle
-- =============================================================================

---Open ONE port with runtime pins; it becomes the active port.
---Ports 1-2 are hardware UARTs; port 3 (serial.SOFT_PORT) is the RMT
---software UART: 5-8 data bits, N/E/O parity, 1-2 stop bits, 300-2000000 baud.
---Calling init again (same or other port) closes the previous port first and
---releases its pins. Fails (false) on ports reserved by the host firmware.
---@param port integer 1, 2, or 3
---@param baud integer e.g. 115200, 921600
---@param rx integer RX GPIO (-1 if unused)
---@param tx integer TX GPIO (-1 if unused)
---@param opts SerialInitOpts?
---@return boolean ok
function serial.init(port, baud, rx, tx, opts) end

---Flush TX, close the active UART, reset both pins to hi-Z GPIO.
---Runs automatically from C when the script ends or crashes.
---@return boolean ok false if nothing was open
function serial.deinit() end

---@return boolean
function serial.is_initialized() end

---Which port is active
---@return integer|nil port 1, 2, 3, or nil when none
function serial.get_port() end

---Move the active UART to new pins on the fly (TX drained first);
---vacated pins are reset to hi-Z.
---@param rx integer
---@param tx integer
---@return boolean ok
function serial.set_pins(rx, tx) end

---Change baud live without reopening the port.
---@param baud integer
---@return boolean ok
function serial.set_baud(baud) end

---@return SerialConfig|nil cfg nil when no port is open
function serial.get_config() end

---Receive error counters since init (soft port only).
---@return SerialStats|nil stats nil unless port 3 is active
function serial.get_stats() end

-- =============================================================================
-- Write
-- =============================================================================

---Binary-safe write; Lua strings may contain NULs.
---Bridge pattern: espnow.on_receive(function(mac, data) serial.write(data) end)
---@param data string
---@return integer bytes_written
function serial.write(data) end

---Write value coerced to text (numbers/booleans fine), no newline.
---@param value any
---@return integer bytes_written
function serial.print(value) end

---serial.print plus "\r\n" (Arduino-style). No argument writes just the newline.
---@param value any?
---@return integer bytes_written
function serial.println(value) end

---Block until every queued TX byte is on the wire.
function serial.flush() end

---Bytes writable right now without blocking (TX backpressure check).
---@return integer
function serial.available_for_write() end

-- =============================================================================
-- Read
-- =============================================================================

---Bytes waiting in the RX buffer.
---@return integer
function serial.available() end

---Non-blocking: up to n buffered bytes (default all) as a binary string;
---"" when nothing is buffered.
---@param n integer?
---@return string data
function serial.read(n) end

---Non-blocking framing: returns a complete chunk (delimiter stripped) once
---the 1-byte delimiter arrives; nil while bytes accumulate internally.
---If the accumulator reaches max (default 512) without a delimiter, the
---whole blob is returned as-is.
---@param delim string single byte, e.g. "\n"
---@param max integer?
---@return string|nil chunk
function serial.read_until(delim, max) end

---Next buffered byte without consuming it.
---@return integer byte -1 when empty
function serial.peek() end

---Discard all buffered RX (including the read_until accumulator).
function serial.clear() end
