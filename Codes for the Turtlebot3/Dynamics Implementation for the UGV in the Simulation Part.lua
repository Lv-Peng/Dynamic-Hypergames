function sysCall_init()
    corout=coroutine.create(coroutineMain)
    path1={0,1,9,3,4,5,6,7,8,10,11,14,12,13,8}--starting from g_1
    path2={3,4,10,11,3,4}--starting from g_5
    k=300/180*math.pi
    
    kv=3
    
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

function coroutineMain()
    
    motorHandles={-1,-1,-1,-1}
    
    vehicleHandle=sim.getObjectHandle('vehicle')
    
    motorHandles[1]=sim.getObjectHandle('joint_front_left_wheel')
    motorHandles[2]=sim.getObjectHandle('joint_front_right_wheel')
    motorHandles[3]=sim.getObjectHandle('joint_back_right_wheel')
    motorHandles[4]=sim.getObjectHandle('joint_back_left_wheel')

    
    path=path1
    
    
    n=table.getn(path)
    i=1
    while true do
        WaypointHandle=sim.getObjectHandle('waypoint_'..path[i])
        targetPos=sim.getObjectPosition(WaypointHandle,vehicleHandle)
        
        v=2
        w=k*math.atan(targetPos[2],targetPos[1])
        if w>2.5 then
            w=2.5
        elseif w<-2.5 then
            w=-2.5
        end
        
        sim.setJointTargetVelocity(motorHandles[1],(v-w)*kv)
        sim.setJointTargetVelocity(motorHandles[2],(-v-w)*kv)
        sim.setJointTargetVelocity(motorHandles[3],(-v-w)*kv)
        sim.setJointTargetVelocity(motorHandles[4],(v-w)*kv)
        
        if math.sqrt(targetPos[2]*targetPos[2]+targetPos[1]*targetPos[1])<0.2 then
            i=i+1
        end
        
        if i>n then
        
            sim.setJointTargetVelocity(motorHandles[1],0)
            sim.setJointTargetVelocity(motorHandles[2],0)
            sim.setJointTargetVelocity(motorHandles[3],0)
            sim.setJointTargetVelocity(motorHandles[4],0)
            break
        end
        
    end
    
end

