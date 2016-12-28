-- MQTT integration for IoT

require("config")

local iot = {}
iot.connected = false

function iot:init()
  self.client = mqtt.Client(CFG.IOT.id, 120, CFG.IOT.user, CFG.IOT.pass, 1)
  self.client:lwt("lwt", NODENAME .. " is offline", 0, 0)
  self.client:on("connect", function(client)
    debug("IoT connected")
    self.connected = true
    self.client:subscribe({["command/" .. NODENAME:lower() .. "/#"] = 1})
    local ssid = wifi.sta.getconfig()
    local ip, nm, gw = wifi.sta.getip()
    local topmsg = {hostname = wifi.sta.gethostname(),
                    mac = wifi.sta.getmac(),
                    ssid = ssid,
                    rssi = wifi.sta.getrssi(),
                    ip = ip,
                    gw = gw}
    self:mpub({wifi = topmsg}, 1, 1, "report/" .. NODENAME:lower())
    local sec, usec = rtctime.get()
    if sec ~= 0 then
      local tm = rtctime.epoch2cal(sec + TZ * 3600)
      local ts = string.format("%04d.%02d.%02d %02d:%02d:%02d",
                                tm["year"], tm["mon"], tm["day"],
                                tm["hour"], tm["min"], tm["sec"])
      self:pub("report/" .. NODENAME:lower() .. "/time", ts, 1, 1)
    end
  end)
  self.client:on("offline", function(client)
    debug("IoT offline")
    self.connected = false
  end)
  self.client:on("message", function(client, topic, msg)
    if msg then
      debug("IoT " .. topic .. ": " .. msg)
      local root, trunk, branch = string.match(topic, '^([^/]+)/([^/]+)/([^/]+)')
      if root == "command" then
        if trunk:lower() == NODENAME:lower() then
          if branch == "restart" then
            node.restart()
          elseif branch == "debug" then
            DEBUG = (msg == "on") and true or false
          elseif branch == "timezone" then
            TZ = tonumber(msg)
          elseif branch == "ntpsync" then
            local server = msg and msg or CFG.NTP.server
            sntp.sync(server)
          end
        end
      end
    end
  end)
end

function iot:connect()
  if wifi.sta.status() == wifi.STA_GOTIP then
    self.client:close()
    self.client:connect(CFG.IOT.server, CFG.IOT.port, CFG.IOT.ssl, CFG.IOT.auto,
    function(client)
      debug("IoT initial connection")
      self.connected = true
    end,
    function(client, reason)
      debug("IoT failed: " .. reason)
      self.connected = false
    end)
  else
    self.connected = false
  end
end

function iot:pub(topic, msg, qos, ret)
  -- Publish the message to a topic
  if not self.connected then
    self:connect()
  elseif wifi.sta.status() == wifi.STA_GOTIP then
    qos = qos and qos or 0
    ret = ret and ret or 0
    msg = msg or ""
    debug("IoT publish: " .. topic .. ": ", msg)
    self.client:publish(topic, msg, qos, ret)
  else
    self.connected = false
  end
end

function iot:mpub(topmsg, qos, ret, btop)
  -- Publish the messages to their topics
  if not self.connected then
    self:connect()
  elseif wifi.sta.status() == wifi.STA_GOTIP then
    qos = qos and qos or 0
    ret = ret and ret or 0
    btop = btop:sub(#btop,#btop) ~= "/" and btop .. "/" or btop
    for topic, msg in pairs(topmsg) do
      if type(msg) == "table" then
        self:mpub(msg, qos, ret, btop .. topic)
      else
        msg = msg or ""
        debug("IoT publish: " .. btop .. topic .. ": ", msg)
        self.client:publish(btop .. topic, msg, qos, ret)
      end
    end
  else
    self.connected = false
  end
end

return iot
-- vim: set ft=lua ai ts=2 sts=2 et sw=2 sta nowrap nu :
