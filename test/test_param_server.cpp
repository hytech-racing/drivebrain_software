#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <queue>
#include <thread>
#include <unordered_set>

#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>

std::atomic<bool> running = true;

int main()
{
    const auto logHandler = [](foxglove::WebSocketLogLevel, char const *msg)
    {
        std::cout << msg << std::endl;
    };

    foxglove::ServerOptions serverOptions;
    serverOptions.capabilities.push_back("parameters");
    serverOptions.capabilities.push_back("parametersSubscribe");
    auto server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>(
        "C++ Protobuf example server", logHandler, serverOptions);
    foxglove::Parameter test_param("test_param", "yo");

    foxglove::ServerHandlers<foxglove::ConnHandle> hdlrs;

    hdlrs.parameterRequestHandler = [&](const std::vector<std::string> &param_names, const std::optional<std::string> &request_id,
                                        foxglove::ConnHandle clientHandle)
    {
        server->publishParameterValues(clientHandle, {test_param}, request_id);
    };
    hdlrs.parameterSubscriptionHandler = [&](const std::vector<std::string> &params_to_subscribe, 
        foxglove::ParameterSubscriptionOperation op_to_perform,
        foxglove::ConnHandle clientHandle) {
        
        std::cout <<"uh"<<std::endl;
    };

    hdlrs.parameterChangeHandler = [&](const std::vector<foxglove::Parameter>& params, const std::optional<std::string>& request_id, foxglove::ConnHandle clientHandle)
    {
        test_param = params.at(0);
        server->publishParameterValues(clientHandle, {test_param}, request_id);
    };
    server->setHandlers(std::move(hdlrs));
    server->start("0.0.0.0", 8765);

    std::signal(SIGINT, [](int sig)
                {
    std::cerr << "received signal " << sig << ", shutting down" << std::endl;
    running = false; });

    while (running)
    {

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    };
    server->stop();
}