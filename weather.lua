-- Weather data

require("config")
iot = require("iot")

local weather = {TMP=nil, DEW=nil, HMD=nil, QFE=nil, QNH=nil, LUX=nil}

function int2float(value, dg)
  local result
  if dg == 2 then
    result = string.format("%d.%02d", value/100, value%100)
  elseif dg == 3 then
    result = string.format("%d.%03d", value/1000, value%1000)
  else
    result = string.format("%d.%02d", value, 0)
  end
  return result
end

function weather:read()
  -- Read the weather sensors
  local bmestat, tslstat = false, false
  -- BME280
  if bme280.init(i2c_sda, i2c_scl) == 2 then
    bmestat = true
    self.QFE, self.TMP = bme280.baro()
    self.QNH = bme280.qfe2qnh(self.QFE, ALTITUDE)
    self.HMD, self.TMP = bme280.humi()
    self.DEW = bme280.dewpoint(hmdt, temp)
  end
  -- TSL2561
  if tsl2561.init(i2c_sda, i2c_scl) == tsl2561.TSL2561_OK then
    tslstat = true
    self.LUX = tsl2561.getlux()
  end
  return bmestat and tslstat
end

function weather:pub()
  -- MQTT publish telemetry data
  local status = self:read()
  if status then
    iot:mpub({temperature = int2float(self.TMP, 2),
              humidity = int2float(self.HMD, 3),
              dewpoint = int2float(self.DEW, 2),
              pressure = int2float(self.QFE, 3),
              sealevel = int2float(self.QNH, 3),
              illuminance = self.LUX},
              0, 0, "sensor/outdoor/")
  end
  iot:mpub({vdd = adc.readvdd33(), heap = node.heap(), uptime = tmr.time()}, 0, 0, "report/" .. NODENAME)
end

function weather:init()
  -- Initialize the weather timer
  tmr.alarm(3, 1000 * weather_interval, tmr.ALARM_AUTO, function() weather:pub() end)
end

return weather
-- vim: set ft=lua ai ts=2 sts=2 et sw=2 sta nowrap nu :
