#pragma once

#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>


// what we want to be able to do here is construct the parameter server from 
// all configuration-needed software components

// REQUIREMENTS:

// - The configuration interface will expose to the parameter server what parameters exist 

// - be able to re-save the final ptree :brain:? (this is something that ROS does not have lol) 

// - the parameter server will also handle recording / tracking of all parameters
//   changed and to what value

// IMPLEMENTATION DETAILS:
// on construction of the parameter server we give it a vector 
// of each configurable-downcasted object. the parameter server looks
// at each of the options that the components contain and creates its 
// vector of foxglove parameters with the correct types

// NOTES:
// the parameter server will also communicate with the mcap broadcaster? 
//  -> yes, and GRPC would be good for this I think for handling the handshake with data_acq
    
    // consideration: why dont we handle the mcap recording within our software here?:
    // at this point why dont I just integrate the mcap recording into here?
        // pros:
        // I wouldnt have to broadcast or handle deserialization 
        // / serialization of the protobuf messages out of the drivebrain core
        
        // cons:
        
        // I would have to then handle all the meta data handling stuff here 
        // too tho which would require alot more integration with foxglove 
        // studio since we would need request-response (service calls) that
        // is already mostly implemented in the data_acq service. 
        
        // I would also have to implement the request / response call for 
        // starting and stopping of recording.