-- INIT, 3 second delay before running main.lua
local IDLE_AT_STARTUP_MS = 3
print("Starting up in " .. IDLE_AT_STARTUP_MS .. " seconds ...")
tmr.alarm(0, IDLE_AT_STARTUP_MS * 1000, tmr.ALARM_SINGLE, function()
  print("Go!")
  dofile("main.lc")
end)
-- vim: set ft=lua ai ts=2 sts=2 et sw=2 sta nowrap nu :
