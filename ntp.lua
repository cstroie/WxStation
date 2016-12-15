-- Time sync using NTP

require("config")

local ntp = {}

function ntp:sync()
  -- Time sync using the specified server, then the gateway
  if wifi.sta.status() == 5 then
    sntp.sync(CFG.NTP.server,
    function(sec, usec, server)
      debug("Time sync to " .. server .. ": " .. sec)
    end,
    function(errcode)
      debug("Time sync failed: " .. errcode)
      local ip, nm, gw = wifi.sta.getip()
      debug("Trying the gateway, " .. gw)
      if gw then sntp.sync(gw) end
    end)
  end
end

function ntp:init()
  -- Init the NTP sync timer
  tmr.alarm(2, 1000 * CFG.NTP.interval, tmr.ALARM_AUTO, function() self:sync() end)
end

return ntp
-- vim: set ft=lua ai ts=2 sts=2 et sw=2 sta nowrap nu :
