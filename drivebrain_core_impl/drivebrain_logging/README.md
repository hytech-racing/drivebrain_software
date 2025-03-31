# overview

the mcap logger handles logging of protobuf and json data. The structred data is logged on call from any other executing thread and on call enqueues a message to be logged by the writer thread. 

the mcap logger is capable of opening and closing mcap files during run-time and upon opening of a new file immediately writes all schemas for all topics / channels that will be logged to. each topic / channel has a specific schema and the schemas are not required to be unique. each schema has an encoding type (json or protobuf) that gets used to tell how to interpret the schema which is required to decode a specific channel's messages.

## protobuf message logging flow



## parameter logging flow

[example writing json messages / schemas](https://github.com/foxglove/mcap/tree/main/cpp/examples/jsonschema)

the mcap logger's parameter logging function gets called at a fixed rate in the parameter logging thread OR when the thread gets notified of parameter change from the foxglove webserver's thread.

the parameters being logged are a subset of the schema that gets written to the mcap file. the schemas is generated from the json file that gets loaded at runtime.

