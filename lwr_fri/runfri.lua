require("rttlib")
rttlib.color=true

tc = rtt.getTC()
depl = tc:getPeer("deployer")

depl:import("lwr_fri")
depl:loadComponent("lwr","lwr_fri::FRIComponent")
depl:setActivity("lwr",0, 1, 0)
lwr = depl:getPeer("lwr")

lwr:getProperty("udp_port"):set(49939)
lwr:configure()
lwr:start()
rtt.logl('Warning', "LWR component started!")