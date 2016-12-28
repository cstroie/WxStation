-- Scan and connect to WiFi

require("config")
ntp = require("ntp")
iot = require("iot")

local wl = {}
wl.try = CFG.WL.tries
wl.status = {"Idle", "Connecting", "Wrong password", "AP not found", "Failed", "Connected"}

function wl:show()
  -- Display WiFi data
  local result = false
  local ssid, password, bssid_set, bssid = wifi.sta.getconfig()
  local ip, nm, gw = wifi.sta.getip()
  if ssid and ip then
    debug(wifi.sta.gethostname(), ssid, wifi.sta.getrssi(), ip, nm, gw)
    result = true
  end
  return result
end

function wl:connect(ssid)
  -- Try to connect and start the watchdog
  debug("WiFi connecting to " .. ssid)
  wifi.sta.config(ssid, CFG.WL.AP[ssid])
  tmr.start(1)
end

function wl.scan(lst)
  -- Check the AP list for a known one
  local ap
  for ssid, v in pairs(lst) do
    authmode, rssi, bssid, channel = string.match(v, "(%d),(-?%d+),(%x%x:%x%x:%x%x:%x%x:%x%x:%x%x),(%d+)")
    debug("  " .. ssid .. " (" .. rssi .. " dBm" .. ")")
    if CFG.WL.AP[ssid] then ap = ssid end
  end
  if ap then
    wl:connect(ap)
  else
    debug("No known WiFi.")
    -- TODO SoftAP mode
  end
end

function wl:check()
  -- Check the WiFi connectivity and try to identify an AP
  if wifi.sta.status() ~= wifi.STA_GOTIP then
    if wl.try > 0 then
      debug(string.format("WiFi wait... % 3d", wl.try), wl.status[wifi.sta.status()])
      wl.try = wl.try - 1
      tmr.start(1)
    else
      debug("WiFi scanning...", "")
      wl.try = CFG.WL.tries
      wifi.setmode(wifi.STATION)
      -- TODO self
      wifi.sta.getap(wl.scan)
    end
  else
    wl.try = CFG.WL.tries
    wl:show()
    ntp:sync()
    iot:connect()
  end
end

-- Start the watchdog
tmr.alarm(1, 1000, tmr.ALARM_SEMI, function() wl:check() end)

return wl
-- vim: set ft=lua ai ts=2 sts=2 et sw=2 sta nowrap nu :
