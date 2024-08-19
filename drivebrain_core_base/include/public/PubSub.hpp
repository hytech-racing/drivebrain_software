#pragma once
#include <string>
#include <boost/signal.hpp>
// this class will contain the implementation of how the components internal to drivebrain software will connect. 

// TODO

// - [ ] protobuf message type agnostic "pub-sub" like communication
    // - how will we handle thread-safe comms? we still want to have non-lossy non-latent comms between
    // - should prob just have the registration of topics with callbacks that get called for each topic name
        // - boost signal registration
    // - the types arent serialized between components, this is only for
    // - we can just have a shared pointer to an instance of the comms backend so that we can have everything in one place
        // - each component can get passed in the shared_ptr and then register their callbacks 
// - [ ] manages buffer of all messages passed through to be sent to the foxglove server and to the mcap recorder


namespace comms
{
    class PubSub
    {
        public:
            PubSub() = default;
            
            template<typename MessageType>
            void register_subscriber(const std::string &topic, std::function<void(const MessageType&)> callback) {}

        private:

    };
}