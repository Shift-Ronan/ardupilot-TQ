-- This script controls a TorqLink thruster via a CAN bus configured to 250 baud

-- Load CAN driver, using the scripting protocol and with a buffer size of 8
local driver = CAN.get_device(8)


function setSpeed(s)
    -- send speed control command

    msg = CANFrame()

    -- extended frame, priority 12, PGN 0xF003, and node ID 208 - (0x8CF003d0)
    -- lua cannot handle numbers so large, so we have to use uint32_t userdata
    msg:id((uint32_t(1) << 31) | (uint32_t(12) << 24) | (uint32_t(tonumber("0xF003")) << 8) | uint32_t(208))

    msg:data(0,255)
    msg:data(1, s) -- speed (0-250)
    msg:data(2,255)
    msg:data(3,255)
    msg:data(4,255)
    msg:data(5,255)
    msg:data(6,255)
    msg:data(7,255)
    -- sending 8 bytes of data
    msg:dlc(8)

    -- write the frame with a 10000us timeout
    driver:write_frame(msg, 10000)

	--gcs:send_text(0,"Sending speed")
end

function setTransmission(t)
    -- set transmission to forward
    msg = CANFrame()

    -- extended frame, priority 12, PGN 0xF005, and node ID 208 - (0x8CF005d0)
    -- lua cannot handle numbers so large, so we have to use uint32_t userdata
    msg:id((uint32_t(1) << 31) | (uint32_t(12) << 24) | (uint32_t(tonumber("0xF005")) << 8) | uint32_t(208))

    msg:data(0,t) -- transmission gear (124-126)
    msg:data(1,255)
    msg:data(2,255)
    msg:data(3,255)
    msg:data(4,255)
    msg:data(5,255)
    msg:data(6,255)
    msg:data(7,255)
    -- sending 8 bytes of data
	msg:dlc(8)

    -- write the frame with a 10000us timeout
    driver:write_frame(msg, 10000)	
	--gcs:send_text(0,"Transmission set")
end

speed = 0
dir = 126
maxTimeout = 10 -- max connection errors in a row
timeout = 0
startTime = 0
function update()
	-- Go back to idle if lost connection
	if not get_frame() then
		timeout = timeout+1
		if timeout > maxTimeout then
			timeout = maxTimeout
			gcs:send_text(0, "Lost connection with TQ")
			speed = 0
			setSpeed(speed)
			setTransmission(125)
			return check_ready()
		end
	else
		timeout = 0
	end
	
	-- Set direction and motor speed
	if speed == 0 then
		setTransmission(125)
	else
		setTransmission(dir)
	end
	setSpeed(speed)
	if speed < 100 then
		speed = speed+1
	end
	return update, 50
end

function get_frame()
	if driver then
		frame = driver:read_frame()
		if frame then
			local id = tostring(frame:id())
			--gcs:send_text(5,string.format("CAN[%u] msg from " .. id .. ": %i, %i, %i, %i, %i, %i, %i, %i", 1, frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
			return frame
		end
	end
	return false
end

function check_ready()
	-- see if we got any frames
	frame = get_frame()
	if frame then
		local id = tostring(frame:id())
		-- TODO: Properly parse CAN ID to check message PGN
		if id == "2566853398" then -- check if Torqeedo is ready by this message
			ready = frame:data(4) >> 7 -- Check first flag of thruster status bitmap

			if ready == 1 then
				gcs:send_text(0, "TQ Ready")
				return update, 2000 -- Need to wait for thruster to be ready
			end
		end
	end

	return check_ready, 10
end

return check_ready()
