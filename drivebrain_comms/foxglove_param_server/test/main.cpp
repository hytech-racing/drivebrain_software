#include <param_server.hpp>


int main()
{
    // with the foxglove websocket api we can pass in our own log handler function. This is what is called a lambda in c++

    const auto logHandler = [](foxglove::WebSocketLogLevel, char const *msg)
    {
        std::cout << msg << std::endl;
    };

    // each server must be created with server options
    foxglove::ServerOptions serverOptions;
    auto server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>(
        "C++ Protobuf example server", logHandler, serverOptions);

    // each operation on the server gets setup by the user here. each handler below with the templated server handlers type. 
    // here is a regular connection handle that is a websocketpp connection handle, not sure if there are any other regular handler
    // types that get used other than this. multiple types of handlers are availbale to be registered within the server handlers and 
    // are specified within the server handlers struct.
    foxglove::ServerHandlers<foxglove::ConnHandle> hdlrs;
     
    hdlrs.subscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle)
    {
        // this here i believe just prints out what client subscribed and doesnt affect any internal state of the server itself
        const auto clientStr = server->remoteEndpointString(clientHandle);
        std::cout << "Client " << clientStr << " subscribed to " << chanId << std::endl;
    };

    // there are 3 different parameter handler functions
    


}