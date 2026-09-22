I'm using this for a quick TODO note. I will clean this up later

* The simulation logic is completely hardcoded in src/humanoid_transport_mujoco/src/mujoco_actuator_transport.cpp. This should be abstracted out so other simulators can be used. 
* Currently, the floor is broadcasted from mujocoactuatortransport, which shouldn't be touching ros broadcasting, so it should be taken out as a separate broadcasting node. 
* Robot has to somehow stand with reduced humanoid_g1 capability. 