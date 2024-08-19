# Drivebrain core messages

these are the protos that manually describe messages for components of the drivebrain. There will also exist generated protobuf message descriptions that may be included by these messages for communication with auto-scaling components of the architecture. 

fixed-scale, or semi-fixed scale components, may describe their messages that they need to send to components within the drivebrain architecture

potential sources of truth for generated protos:

1. CAN dbc file (exists, and is within the `drivebrain_comms` layer)
2. comms champ dsl (`drivebrain_comms` layer)
3. simulink/matlab (`drivebrain_control` and `drivebrain_state_estimation` layer)

