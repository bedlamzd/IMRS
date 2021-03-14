function sysCall_init()
    graph = sim.getObjectHandle('Graph')
    sensor = sim.getObjectHandle('left_holder_sensor')
    buffer = {}
    max_points = 20
end

function sumf(a, ...) return a and a + sumf(...) or 0 end
function sumt(t) return sumf(unpack(t)) end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    res, d = sim.readProximitySensor(sensor)
    if res > 0 then
        if #buffer >= max_points then
            table.remove(buffer, 1)
        end
        buffer[#buffer + 1] = d
        data = sumt(buffer)
        sim.setGraphUserData(graph, 'transformed_data', data / #buffer)
    end
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
