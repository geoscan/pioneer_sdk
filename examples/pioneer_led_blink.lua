-- Simplification and caching table.unpack calls
local unpack = table.unpack

-- Base pcb number of RGB LEDs
local ledNumber = 4
-- RGB LED control port initialize
local leds = Ledbar.new(ledNumber)

-- Function changes color on all LEDs
local function changeColor(color)
    -- Changing color on each LED one after another
    for i=0, ledNumber - 1, 1 do
        leds:set(i, unpack(color))
    end
end

-- Event processing function called automatically by autopilot
function callback(event)

end

changeColor({.5, .5, .5})
sleep(1)
changeColor({0, 0, 0})
