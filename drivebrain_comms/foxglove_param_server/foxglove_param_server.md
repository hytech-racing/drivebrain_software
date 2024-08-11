# parameter server

this component of the drivebrain will contain the foxglove parameter server implementation / intergration for handling the adjustment of internal parameters.

what we want to be able to do here is construct the parameter server from all configuration-needed software components

REQUIREMENTS:

- The configuration interface will expose to the parameter server what parameters exist 

- be able to re-save the final ptree :brain:? (this is something that ROS has lol)

- the parameter server will also handle recording / tracking of all parameters changed and to what value

## IMPLEMENTATION DETAILS

on construction of the parameter server we give it a vector of each configurable-downcasted object. the parameter server looks at each of the options that the components contain and creates its vector of foxglove parameters with the correct types

all parameters will be tracked within a generated protobuf messages that each get populated initially on construction and are owned by this parameter server.

when a parameter change occurs, the protobuf message that contains all of the parameters for that component getting changed gets updated as well and then the parameter server puts it onto the queue to be sent out to the mcap recorder and the foxglove 

## NOTES
the parameter server will also communicate with the mcap broadcaster? 
- yes, and GRPC would be good for this I think for handling the handshake with `data_acq` 
    - well, in reality I think we should only implement this if just assuming that `data_acq` is recording becomes a bad assumption. 
    
### CONSIDERATIONS 

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