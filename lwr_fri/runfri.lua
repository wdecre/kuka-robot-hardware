require("rttlib")
rttlib.color=true

tc = rtt.getTC()
depl = tc:getPeer("deployer")

depl:import("lwr_fri")
depl:import("rtt_ros_service")
depl:loadComponent("lwr","lwr_fri::FRIComponent")
depl:setActivity("lwr",0, 1, 0)
lwr = depl:getPeer("lwr")

lwr:getProperty("udp_port"):set(49938)
lwr:configure()
lwr:start()
rtt.logl('Warning', "LWR component started!")

cp = rtt.Variable("ConnPolicy")
cp.name_id="/fri_jnt_impedance"
cp.transport=3
depl:stream("lwr.FriJointImpedance", cp)

cp.name_id="/jnt_position"
depl:stream("lwr.JointPositionCommand", cp)
cp.name_id="/jnt_velocity"
depl:stream("lwr.JointVelocityCommand", cp)
cp.name_id="/jnt_effort"
depl:stream("lwr.JointEffortCommand", cp)

cp.name_id="/jnt_state"
depl:stream("lwr.JointState", cp )
