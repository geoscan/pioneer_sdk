-- This script serves the demonstration of a "compile/upload" functionality
-- implemented w/ use of MAVLink FTP subprotocol running on both UAV and
-- client sides. In the case of a successful attemp (it may not be the case
-- if you use incorrect parameters, or an outdated version of the UAV's
-- firmware), the demo script will compile, upload, launch this Lua script,
-- and as a result, the copter will make a short blink.
--
-- Please make sure that you use a battery, as USB power supply may not
-- be sufficient for LEDs.

-- Simplification and caching table.unpack calls
local unpack = table.unpack

-- Both Pioneer and Pioneer Mini have 4 RGB LEDs.
local ledNumber = 4
-- Create an object that encapsulates working w/ LEDs behind a convenient interface
local leds = Ledbar.new(ledNumber)

-- The function sets brightness and color for each of the 4 LEDs
local function changeColor(color)
    for i=0, ledNumber - 1, 1 do
        leds:set(i, unpack(color))
    end
end

-- Event processing function where you can handle Autopilot-generated events.
-- Not needed in this case
function callback(event)
end

changeColor({.5, .5, .5})  -- Set white color on all LEDs, half of the max brightness
sleep(1)  -- Hold for a second
changeColor({0, 0, 0})  -- Disable LEDs
