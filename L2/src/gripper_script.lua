function sysCall_init()
    simRemoteApi.start(19999)
    left_holder = sim.getObjectHandle('left_holder_joint')
    right_holder = sim.getObjectHandle('right_holder_joint')
    sensor = sim.getObjectHandle('left_holder_sensor')
    D_EPS = 1e-4
    velocity = 0.008
    wait_interval = 5

    wait_start = nil
    init_distance = nil
    clenched = nil
    waited = nil
    unclenched = nil
end

function clench()
    if clenched then
        return
    end
    res, distance = sim.readProximitySensor(sensor)
    if res > 0 then
        if init_distance == nil then
            init_distance = distance
        end
        if distance <= D_EPS then
            sim.setJointTargetVelocity(left_holder, 0)
            sim.setJointTargetVelocity(right_holder, 0)
            clenched = true
            print('Done.')
            return
        end
    end
    sim.setJointTargetVelocity(left_holder, velocity)
    sim.setJointTargetVelocity(right_holder, velocity)
    print('Clenching...')
end

function wait()
    if waited or not clenched then
        return
    elseif start == nil then
        start = sim.getSimulationTime()
    elseif sim.getSimulationTime() - start > wait_interval then
        waited = true
        print('Done.')
        return
    end
    print('Waiting...')
end

function unclench()
    if unclenched or not waited then
        return
    end

    res, distance = sim.readProximitySensor(sensor)
    if res > 0 and math.abs(distance - init_distance) <= D_EPS then
        sim.setJointTargetVelocity(left_holder, 0)
        sim.setJointTargetVelocity(right_holder, 0)
        unclenched = true
        print('Done.')
        return
    end
    sim.setJointTargetVelocity(left_holder, -velocity)
    sim.setJointTargetVelocity(right_holder, -velocity)
    print('Unclenching...')
end


function getSimTime_function(inInts,inFloats,inStrings,inBuffer)
    return {}, {sim.getSimulationTime()}, {}, ''
end

function sysCall_actuation()
    clench()
    wait()
    unclench()
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
