-- MQTT integration for IoT

require("config")

local iot = {}
iot.connected = false
iot.queue = {}
iot.mqueue = {}

function iot:init()
  self.client = mqtt.Client(iot_id, 120, iot_user, iot_pass, 1)
  self.client:lwt("lwt", NODENAME .. " is offline", 0, 0)
  self.client:on("connect", function(client)
    debug("IoT connected")
    self.connected = true
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
        if trunk == "rcs" then
          local rcs = require("rcs")
          rcs:button(branch, msg)
          unrequire("rcs")
        elseif trunk == NODENAME then
          if branch == "restart" then
            node.restart()
          elseif branch == "debug" then
            DEBUG = (msg == "on") and true or false
          elseif branch == "timezone" then
            timezone = msg
          elseif branch == "ntpsync" then
            local server = msg and msg or ntp_server
            sntp.sync(server)
          elseif branch == "beep" then
            local beep = require("beep")
            beep:onekhz()
            unrequire("beep")
          elseif branch == "light" then
            lcd_bl = (msg == "on") and true or false
          end
        end
      end
    end
  end)
end

function iot:connect()
  if wifi.sta.status() == 5 then
    self.client:close()
    self.client:connect(iot_server, iot_port, 0, 1,
    function(client)
      debug("IoT initial connection")
      self.connected = true
      self.client:subscribe({["command/#"] = 1})
      local ssid = wifi.sta.getconfig()
      local ip, nm, gw = wifi.sta.getip()
      local topmsg = {hostname = wifi.sta.gethostname(), mac = wifi.sta.getmac(),
                      ssid = ssid, rssi = wifi.sta.getrssi(), ip = ip, gw = gw}
      local sec, usec = rtctime.get()
      iot:mpub({wifi = topmsg, time = sec}, 1, 1, "report/" .. NODENAME)
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
    --table.insert(iot.queue, {topic, msg, qos, ret})
    self:connect()
  elseif wifi.sta.status() == 5 then
    qos = qos and qos or 0
    ret = ret and ret or 0
    msg = msg or ""
    debug("IoT publish: " .. topic .. ": ", msg)
    self.client:publish(topic, msg, qos, ret)
  end
end

function iot:mpub(topmsg, qos, ret, btop)
  -- Publish the messages to their topics
  if not self.connected then
    self:connect()
  elseif wifi.sta.status() == 5 then
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
  end
end

return iot
-- vim: set ft=lua ai ts=2 sts=2 et sw=2 sta nowrap nu :
