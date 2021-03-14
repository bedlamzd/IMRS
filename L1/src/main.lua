function sysCall_init()
    left_holder = sim.getObjectHandle("left_holder_joint")
    right_holder = sim.getObjectHandle("right_holder_joint")
    sensor = sim.getObjectHandle("left_holder_sensor")
    min_gap = 1e-4 -- minimal gap that should be between sensor and object
    dist_thresh_to_grip = 50e-3 / 2 -- distance after which we decide to grab object
    joint_grip_velocity = 0.005
    print(min_gap)
    print(dist_thresh_to_grip)
    print(joint_grip_velocity)
end

function sysCall_actuation()
    result, distance = sim.readProximitySensor(sensor)
    if result > 0 then
        stop = (distance <= min_gap)
        grab = (distance <= dist_thresh_to_grip)
        print(distance)
        print(grab)
        print(stop)
        if grab and not stop then
            sim.setJointTargetVelocity(left_holder, joint_grip_velocity)
            sim.setJointTargetVelocity(right_holder, joint_grip_velocity)
        else
            sim.setJointTargetVelocity(left_holder, 0)
            sim.setJointTargetVelocity(right_holder, 0)
        end

    end
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
