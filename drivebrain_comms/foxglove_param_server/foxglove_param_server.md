# parameter server


what we want to be able to do here is construct the parameter server from all configuration-needed software components

REQUIREMENTS:

- The configuration interface will expose to the parameter server what parameters exist 

- be able to re-save the final ptree :brain:? (this is something that ROS has lol)

- the parameter server will also handle recording / tracking of all parameters changed and to what value

## IMPLEMENTATION DETAILS

on construction of the parameter server we give it a vector of each configurable-downcasted object. the parameter server looksat each of the options that the components contain and creates its vector of foxglove parameters with the correct types

## NOTES
the parameter server will also communicate with the mcap broadcaster? 
- yes, and GRPC would be good for this I think for handling the handshake with `data_acq`
    
### considerations 

why dont we handle the mcap recording within our software here?:
- at this point why dont I just integrate the mcap recording into here?

-------
##### pros:
- I wouldnt have to broadcast or handle deserialization / serialization of the protobuf messages out of the drivebrain core

- I wouldnt have to implement a protocol to handle handshaking with an external 
-------
##### cons:  
- I would have to then handle all the meta data handling stuff here too tho which would require alot more integration with foxglove studio since we would need request-response (service calls) that is already mostly implemented in the `data_acq` service. 
       
- I would also have to implement the request / response call for starting and stopping of recording.
-------

this component of the drivebrain will contain the foxglove parameter server implementation / intergration for handling the adjustment of internal parameters.

## TODO
- [ ] get a simple parameter being able to be changed from foxglove via this implementation
- [ ] figure out how we can connect a boost ptree getting registered during runtime to 