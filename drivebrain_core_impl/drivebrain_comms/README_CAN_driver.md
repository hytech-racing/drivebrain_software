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