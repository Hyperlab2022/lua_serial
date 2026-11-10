-- soft_serial_loopback.lua — loopback test for the RMT software UART (port 3)
--
-- Wire TX_PIN to RX_PIN with a jumper, then run this script on the device.
-- It sends FRAMES_PER_SEC framed packets of FRAME_LEN bytes for SECONDS,
-- reads them back, and checks every frame (sync + sequence + checksum).
--
-- Try it while the device is busy too (BLE connected, file upload running)
-- to see how flash writes affect RX: the stats line counts the drops.

local RX_PIN, TX_PIN = 16, 15     -- any free GPIOs (not 0/3/45/46, 19/20, 35-37)
local BAUD = 115200
local FRAME_LEN = 100             -- 2 sync + 2 seq + payload + 2 checksum
local FRAMES_PER_SEC = 30
local SECONDS = 20

assert(serial.init(serial.SOFT_PORT, BAUD, RX_PIN, TX_PIN, {rxbuf = 4096}),
       "soft port init failed")

local SYNC = "\165\090"           -- 0xA5 0x5A
local PAYLOAD_LEN = FRAME_LEN - 6

local function checksum(s, from, to)
  local sum = 0
  for i = from, to do sum = (sum + s:byte(i)) & 0xFFFF end
  return sum
end

local function make_frame(seq)
  local t = {}
  for i = 1, PAYLOAD_LEN do t[i] = (seq * 7 + i * 13) & 0xFF end
  local body = string.char((seq >> 8) & 0xFF, seq & 0xFF) .. string.char(table.unpack(t))
  local sum = checksum(body, 1, #body)
  return SYNC .. body .. string.char((sum >> 8) & 0xFF, sum & 0xFF)
end

local total = FRAMES_PER_SEC * SECONDS
local period = 1000 / FRAMES_PER_SEC
local sent, good, bad, short_writes, skipped = 0, 0, 0, 0, 0
local buf = ""
local start = millis()

local function parse()
  while true do
    local p = buf:find(SYNC, 1, true)
    if not p then
      -- keep a trailing 0xA5 that may be the first half of a sync
      local keep = (buf:sub(-1) == "\165") and 1 or 0
      skipped = skipped + #buf - keep
      buf = buf:sub(#buf - keep + 1)
      return
    end
    if p > 1 then
      skipped = skipped + p - 1
      buf = buf:sub(p)
    end
    if #buf < FRAME_LEN then return end
    local sum = (buf:byte(FRAME_LEN - 1) << 8) | buf:byte(FRAME_LEN)
    if checksum(buf, 3, FRAME_LEN - 2) == sum then
      good = good + 1
      buf = buf:sub(FRAME_LEN + 1)
    else
      bad = bad + 1
      buf = buf:sub(3)             -- drop this sync, hunt for the next one
    end
  end
end

while true do
  local now = millis()
  if sent < total and now - start >= sent * period then
    local f = make_frame(sent)
    if serial.write(f) ~= #f then short_writes = short_writes + 1 end
    sent = sent + 1
  end

  local data = serial.read()
  if #data > 0 then
    buf = buf .. data
    parse()
  end

  if sent >= total and (good + bad >= total or now - start > SECONDS * 1000 + 2000) then
    break
  end
  delay(1)
end

local st = serial.get_stats() or {}
print(string.format(
  "soft serial %d baud: sent %d, good %d, bad %d, lost %d, skipped bytes %d, short writes %d",
  BAUD, sent, good, bad, sent - good - bad, skipped, short_writes))
print(string.format("rx stats: overflows %d, frame errors %d, parity errors %d",
  st.rx_overflows or -1, st.rx_frame_errors or -1, st.rx_parity_errors or -1))
serial.deinit()
