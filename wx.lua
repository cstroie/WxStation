-- Read weather data from sensors and publish it

require("config")
iot = require("iot")

local wx = {TMP=nil, DEW=nil, HMD=nil, QFE=nil, QNH=nil, LUX=nil, LVS=nil, LIR=nil}

function int2float(x, pr)
  -- Convert integer to "float" with specified precision
  local result
  if pr == 2 then
    result = string.format("%d.%02d", x/100, x%100)
  elseif pr == 3 then
    result = string.format("%d.%03d", x/1000, x%1000)
  else
    result = string.format("%d.%02d", x, 0)
  end
  return result
end

function wx:read()
  -- Read the weather sensors
  local bmests, tslsts = false, false
  -- BME280
  if bme280.init(I2C_SDA, I2C_SCL) == 2 then
    bmests = true
    self.QFE, self.TMP = bme280.baro()
    self.QNH = bme280.qfe2qnh(self.QFE, ALT)
    self.HMD, self.TMP = bme280.humi()
    self.DEW = bme280.dewpoint(self.HMD, self.TMP)
  end
  -- TSL2561
  if tsl2561.init(I2C_SDA, I2C_SCL) == tsl2561.TSL2561_OK then
    tslsts = true
    self.LUX = tsl2561.getlux()
    self.LVS, self.LIR = tsl2561.getrawchannels()
  end
  return bmests and tslsts
end

function wx:pub()
  -- MQTT publish telemetry data
  local status = self:read()
  if status then
    iot:mpub({temperature = int2float(self.TMP, 2),
              humidity = int2float(self.HMD, 3),
              dewpoint = int2float(self.DEW, 2),
              pressure = int2float(self.QFE, 3),
              sealevel = int2float(self.QNH, 3),
              illuminance = self.LUX,
              visible = self.LVS,
              infrared = self.LIR},
              0, 0, "sensor/outdoor/")
  end
  iot:mpub({vdd = adc.readvdd33(),
            heap = node.heap(),
            rssi = wifi.sta.getrssi(),
            uptime = tmr.time()},
            0, 0, "report/" .. NODENAME:lower())
end

function wx:init()
  -- Initialize the weather timer
  tmr.alarm(3, 1000 * CFG.WX.interval, tmr.ALARM_AUTO, function() self:pub() end)
end

return wx
-- vim: set ft=lua ai ts=2 sts=2 et sw=2 sta nowrap nu :
