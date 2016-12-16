-- Main file, run by init.lua, after a short delay

function debug(...)
  -- Print the message if the global DEBUG flag is on
  if DEBUG then print(...) end
end

-- Use ADC to read Vdd
if adc.force_init_mode(adc.INIT_VDD33) then node.restart() end

-- NTP sync
ntp = require("ntp")
ntp:init()

-- Weather data
wx = require("wx")
wx:init()

-- IoT
iot = require("iot")
iot:init()

-- Wireless
wl = require("wl")

-- Global debug flag
--DEBUG = false

-- vim: set ft=lua ai ts=2 sts=2 et sw=2 sta nowrap nu :
