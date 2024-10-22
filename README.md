## quickstart

### native platform nix build:
`nix build`

### development building
```nix develop```

```build```

### cross-compile natively (if on x86)
`nix build .#legacyPackages.x86_64-linux.pkgsCross.aarch64-multiplatform.drivebrain_software`

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

## release plan
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