## design

[wiki link](https://wiki.hytechracing.org/books/software/page/drivebrain-architecture-rev1)

## implementation details

- `drivebrain_core_base` shouldnt change from car to car but `drivebrain_core_impl` may change


## first pass
ideas:
- at first, we will generate protobuf message descriptions from the simulink model
- we will do handle manual integration for any new inputs / outputs

questions:
1. do we want to separate the state estimation and the controllers?
    - I think so since we can iterate on controllers faster than state estimation and switch between them using the same plugin/library defined state estimation systems.

2. does it really make sense to have the state estimation be separated from the controller when they are both developed in simulink?
    - maybe...?
    - how will we handle having both the state estimation and the controller in simulink lap sim AND having to have wrapper code for when it becomes a separate lib integration?

I think for q2, we can go with keeping the state estimation monolithic? 

- keep CASE, just new wrapper code for v1? 

## prototyping

### ideas about manually implemented controllers vs generated controllers
idea: if we have the ability to go both ways for having the ability to both:
 
1. define/add inputs for a simulink model based on a .proto message

2. generate a .proto message from either inputs OR outputs of a simulink model

and generated code for the data passing into and out of proto messages, then we can book-end the controllers and estimators. 


### release planning

#### release notes:
- verified that we can have both drivebrain and the data_acq bound to the same CAN device and receiving / talking over it
    - due to this, we dont need to have the live telem in the alpha since we will only be using CAN still for this release for all comms

- [x] alpha feature set ((start: 8/19) ~2 weeks, 16 days): -> basic control of car with TCU
- [x] basic controller library (7 days) (first pass I want to try out different types of regen handling)
    - [x] live parameter controller interface (2 days, 50%)
    - [x] simple controller business logic (1 day)
    - [x] basic state estimator that can be sampled and manages state (2 days)
- [x] create the state estimation system that can hold multiple state estimators (2 days)
    - [x] create a state of the car that can be updated by each of the drivers (1 day)
    - [x] integrate the driver inputs into state estimator for creating a shared state (2 days)
- [x] CAN MCU driver library (4 days)
    - [x] DBC based parsing 
    - [x] async receiving and transmitting with Boost.Asio (2 days)
    - [x] protobuf message packing (2 days)
    - [x] simple internal communication with basic controller (1 day)
- [x] application runtime (1 day)
- [x] foxglove live parameter server and websocket integration (1 day)

beta feature set (): -> existing controller on TCU with telemetry and recording

(WEEK 1):
- [ ] improve protobuf generation from DBC by supporting enums properly (1 day)
    - tied to making the DBC parser also able to handle enums
- [ ] CAN driver improvements (1 day)
    - [ ] make the DBC parser also be able to handle enums
- [ ] controller manager implementation (4 days)
    - [ ] be able to switch between controllers in a safe manor while driving (2 days, 50% done)
    - [ ] hook into the grpc server through some method for handling the switch (2 days)
- [ ] grpc server interface for switching between controllers (1 day)

(WEEK 2)
- [ ] vectornav UART driver integration that adds new messages to the message bus (1 week)

(WEEK 3):
- [ ] implement MCAP recording of all messages between components (1 week)
    - [ ] implement MCAP recording componenet (3 days) 
    - [ ] driver inputs into state estimator (1 day)
    - [ ] state estimation output (1 day)
    - [ ] controller output to the car (1 day)

(WEEK 4):
- [ ] add CASE to controllers available to switch to (3 days)
- [ ] implement live telemetry of all messages (4 days)
    - [ ] add to the foxglove server interface the ability to live output protobuf messages (2 days)
    - [ ] hookup all components (2 days)

v1 feature set (3 weeks): -> 
- [ ] LQR based controller integrated onto car for torque vectoring (2 weeks)
- [ ] luke's models integrated into state estimator (1 week)

v1.1 feature set (): -> ht09-capable 
- [ ] be able  

### TODOs
- [ ] make a controller manager for switching between controllers
- [ ] implement simple controller to use for testing
    - end to end test of the controller?
- [ ] write tests
    - will obv want to also have these run in CI (`nix develop` devshell works pretty well for this and we can use the same testing framework that is used by our platformio stuff for easy of knowledge sharing)
- [ ] implement the core protocol over UART for the controller to talk directly to the micro
    - the output of this would also benefit from the commschamp to protobuf message adapter since we want to be able to log this output by default
        - these driver comms between the micro
- [ ] make an interface for the foxglove parameter server 
    -  this will also need to talk with both the controller manager and the
    - how will we hook this up into the controller and state estimator?

### resources / libs to use

(potential) for CAN interaction: https://github.com/SimonCahill/libsockcanpp

for CAN parsing: https://github.com/xR3b0rn/dbcppp

## what we want from the drivebrain driver bus

how much do we want to tie in protobuf?
    - do we want each driver to have to deal with protobufs or do we want them to output something else?
        - we need a way to unify each driver's output to the driver bus 
            - I think it would be fine to have each driver be able to define their own protobuf message for output
                - we cant have a single message type that we compose from the driver messages
                - we will need some sort of registration for each message type
            - each driver can act as an adapter to protobuf messages
            - do we want to be able to have N messages be returned from each driver? or does each driver have to only output one message?
                - we still want to be able to have a single message per topic in foxglove / mcap
                - we can have named channels in each driver 
                - protobuf introspection for the data acq

what about a json based message struct generator for setting what each driver can output?
    - we can generate the json with matlab too and so we can hook into the code gen 
    - we can generate protos and adapter code between the struct and the resulting protobuf message to 
output requirements:
- needs to be composed directly from drivers it is composed of
    - the drivers being used are known at compile time
- parameter interface
    - the drivers have to a parameter update function that can be used outside of initialization

## development

`nix develop` enters you into the devshell for building with cmake

example `c_cpp_properties.json` file to be placed in your `.vscode/`:
```json
{
    "configurations": [
        {
            "name": "dev",
            "includePath": [
                "${workspaceFolder}/**"
            ],
            "defines": [],
            "compileCommands": "${workspaceFolder}/build/compile_commands.json"
        }
    ],
    "version": 4
}
```