# Drivebrain communications

each one of these drivers will talk with either a generated protobuf library or one they construct themselves.

## parameter server

this component of the drivebrain will contain the foxglove parameter server implementation / intergration for handling the adjustment of internal parameters.

what we want to be able to do here is construct the parameter server from all configuration-needed software components

REQUIREMENTS:

- The configuration interface will expose to the parameter server what parameters exist 

- be able to re-save the final ptree :brain:? (this is something that ROS has lol)

- the parameter server will also handle recording / tracking of all parameters changed and to what value

### IMPLEMENTATION DETAILS

on construction of the parameter server we give it a vector of each configurable-downcasted object. the parameter server looks at each of the options that the components contain and creates its vector of foxglove parameters with the correct types

all parameters will be tracked within a generated protobuf messages that each get populated initially on construction and are owned by this parameter server.

when a parameter change occurs, the protobuf message that contains all of the parameters for that component getting changed gets updated as well and then the parameter server puts it onto the queue to be sent out to the mcap recorder and the foxglove 

### NOTES
the parameter server will also communicate with the mcap broadcaster? 
- yes, and GRPC would be good for this I think for handling the handshake with `data_acq` 
    - well, in reality I think we should only implement this if just assuming that `data_acq` is recording becomes a bad assumption. 
    
#### CONSIDERATIONS 

#### why dont we handle the mcap recording within our software here?
- at this point why dont I just integrate the mcap recording into here?


-------
###### pros:
- I wouldnt have to broadcast or handle deserialization / serialization of the protobuf messages out of the drivebrain core

-------
###### cons:  
- I would have to then handle all the meta data handling stuff here too tho which would require alot more integration with foxglove studio since we would need request-response (service calls) that is already mostly implemented in the `data_acq` service. 
       
- I would also have to implement the request / response call for starting and stopping of recording.
-------


#### why not also have this foxglove server broadcast out all of the mcap messages being received?

-------
###### pros:
- we wouldnt have to have another service running on the pi
- we wouldnt have to serialize, de-serialize and then re-serialize the same protobuf msg 

-------
###### cons:  
- more work within this parameter server to achieve the same results that we currently have
- reduces separation of concerns even more (codebase gets more messy and becomes more grown instead of designed)


-------

## TODO
- [x] get a simple parameter being able to be changed from foxglove via this implementation
- [ ] figure out how we can connect a boost ptree getting registered during runtime 
- [ ] implement the registering of the configurable components and test it out with real ptree

## CAN driver

### assumptions
- we can assume that this is running only on a linux machine that has access to socketcan 


### requirements

- [ ] all system driver level components must be abstracted from the message conversion to enable SITL unit testing of message conversion

- [ ] has to be able to output the current parsed CAN messages in the generated protobuf messages that we currently

    - some notes:
        - [this](https://github.com/hytech-racing/data_acq/blob/master/py_dbc_proto_gen/dbc_to_proto.py) is the current way we are generating the protobuf message description we currently have

        - [this](https://github.com/hytech-racing/data_acq/blob/master/dbc_proto_bin_gen.nix#L13) is where the protobuf script gets ran in the `data_acq` nix architecture to output the protobuf

        - [this](https://github.com/hytech-racing/data_acq/blob/master/flake.nix#L65) is where the protobuf python library gets generated in the nix architecture for the creation of the protobuf generated encoding / decoding lib
            - [`nix-proto` repository](https://github.com/notalltim/nix-proto)

    
- [ ] has to use the existing `.dbc` CAN description to parse all of the messages and be able to have this change and not be hard-coded to a specific `.dbc` version 

- [ ] has to have a handler for sending any of the CAN messages in the `.dbc` file. preferrably this interface is via the same protobuf message that it sends out per-each parsed CAN message so that path way is "reversable" (protobuf-to-CAN and CAN-to-protobuf msgs)

- [ ] must not use polling to parse or do any busy waiting to handle parsing. it must awake its thread and handle via callback the parsing for efficient resource utilization

- [ ] must be configurable to be able to use a virtual can interface. ie: must be testable with our [broadcast-test.py script](https://github.com/hytech-racing/data_acq/blob/master/py_data_acq/broadcast-test.py)

    - this enables user integration testing without access to the car

### resources or potentially useful links

https://github.com/djarek/canary/tree/master

https://elinux.org/Bringing_CAN_interface_up

https://www.kernel.org/doc/html/next/networking/can.html

[example version of our generated `.proto`](https://github.com/hytech-racing/data_acq/releases/download/2024-04-27T00_26_50/hytech.proto)

[most recent `hytech.dbc`](https://github.com/hytech-racing/HT_CAN/releases/download/109/hytech.dbc)
