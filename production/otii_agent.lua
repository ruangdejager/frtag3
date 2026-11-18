--[[
otii_agent.lua

Otii Arc power-control agent for production/provision_tag.ps1.

Run by otiicli.exe (Otii Automation Toolbox licence required):

    otiicli.exe -n otii_agent.lua <pipe name>

It opens the first Otii Arc it finds, prints "READY" on stdout, and then serves
one-line commands that the PowerShell script writes to a local server (a
Windows named pipe called <pipe name>). Every command is answered with exactly
one stdout line starting with OK, MEAS or ERR, so the script can pair answers
with commands without parsing anything else otiicli prints.

    volt <V>      set the main output voltage                 -> OK
    on | off      switch the main output                       -> OK
    meas <ms>     record the main current channel for <ms>,
                  return statistics in amperes                 -> MEAS avg=<A> min=<A> max=<A> n=<samples>
    exit          switch the output off and end the agent      -> OK

Built only from calls Qoitech's own example scripts use (examples/CI.lua,
event_loop.lua, capture_to_csv.lua in the Otii install folder), because the
agent cannot be exercised without an Arc and a licence on the development PC.
If an Otii release renames one of them, the agent answers ERR and the script
carries on without power data rather than failing the unit.
--]]

otii.clear()

local args = otii.get_args()
assert(#args >= 2, "usage: otii_agent.lua <pipe name>")
local pipeName = args[2]

local devices = otii.get_devices("Arc")
assert(#devices > 0, "No Otii Arc found")
local box = otii.open_device(devices[1].id)
assert(box ~= nil, "Cannot open the Otii Arc")

local project = otii.get_active_project()
if project == nil then
    project = otii.create_project()
end
assert(project ~= nil, "Cannot open or create an Otii project")

box:enable_channel("mc", true)
box:enable_main(false)

local function measure(ms)
    project:start()
    otii.msleep(ms)
    project:stop()

    local rec = project:get_last_recording_id()
    local count = project:get_channel_count(box, rec, "mc")
    if count == nil or count < 1 then
        return "ERR no samples recorded"
    end
    local data = project:get_channel_data(box, rec, "mc", 1, count)
    local sum, min, max = 0, math.huge, -math.huge
    for _, item in ipairs(data) do
        local v = item.sample
        sum = sum + v
        if v < min then min = v end
        if v > max then max = v end
    end
    return string.format("MEAS avg=%.9g min=%.9g max=%.9g n=%d", sum / #data, min, max, #data)
end

local function handle(line)
    local cmd, arg = line:match("^%s*(%a+)%s*(%S*)")
    if cmd == "volt" then
        box:set_main_voltage(tonumber(arg))
        return "OK"
    elseif cmd == "on" then
        box:enable_main(true)
        return "OK"
    elseif cmd == "off" then
        box:enable_main(false)
        return "OK"
    elseif cmd == "meas" then
        return measure(tonumber(arg))
    elseif cmd == "exit" then
        box:enable_main(false)
        otii.writeln("OK")
        otii.stop(0)
        return nil
    end
    return "ERR unknown command: " .. tostring(line)
end

local server = otii.create_local_server(pipeName, function(data)
    local ok, reply = pcall(handle, data)
    if not ok then
        reply = "ERR " .. tostring(reply)
    end
    if reply ~= nil then
        otii.writeln(reply)
    end
end)
assert(server ~= nil, "Cannot create local server " .. pipeName)

otii.writeln("READY")
otii.run()

server:close()
box:close()
