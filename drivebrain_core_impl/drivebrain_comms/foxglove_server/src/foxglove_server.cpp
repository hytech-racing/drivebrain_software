#include <foxglove_server.hpp>
#include <variant>
#include <hytech_msgs.pb.h>

#include <queue>
#include <unordered_set>

#include <string>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>

#include <algorithm>

static uint64_t nanosecondsSinceEpoch()
{
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count());
}

/// Builds a FileDescriptorSet of this descriptor and all transitive dependencies, for use as a
/// channel schema.
static std::string SerializeFdSet(const google::protobuf::Descriptor *toplevelDescriptor)
{
    google::protobuf::FileDescriptorSet fdSet;
    std::queue<const google::protobuf::FileDescriptor *> toAdd;
    toAdd.push(toplevelDescriptor->file());
    std::unordered_set<std::string> seenDependencies;
    while (!toAdd.empty())
    {
        const google::protobuf::FileDescriptor *next = toAdd.front();
        toAdd.pop();
        next->CopyTo(fdSet.add_file());
        for (int i = 0; i < next->dependency_count(); ++i)
        {
            const auto &dep = next->dependency(i);
            if (seenDependencies.find(dep->name()) == seenDependencies.end())
            {
                seenDependencies.insert(dep->name());
                toAdd.push(dep);
            }
        }
    }
    return fdSet.SerializeAsString();
}

core::FoxgloveWSServer::FoxgloveWSServer(std::vector<core::common::Configurable *> configurable_components, deqtype &out_queue) : _components(configurable_components), _out_queue(out_queue)
{
    _log_handler = [](foxglove::WebSocketLogLevel, char const *msg)
    {
        std::cout << msg << std::endl;
    };

    _server_options.capabilities.push_back("parameters");

    _server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>(
        "T.A.R.S", _log_handler, _server_options);

    foxglove::ServerHandlers<foxglove::ConnHandle> hdlrs;

    hdlrs.parameterRequestHandler = [this](const std::vector<std::string> &param_names, const std::optional<std::string> &request_id,
                                           foxglove::ConnHandle clientHandle)
    {
        auto params_vec = _get_current_params();
        _server->publishParameterValues(clientHandle, params_vec, request_id);
    };

    hdlrs.parameterChangeHandler = [&](const std::vector<foxglove::Parameter> &params, const std::optional<std::string> &request_id, foxglove::ConnHandle clientHandle)
    {
        // loop through all of the params we are trying to change
        // auto name_type_map = _get_current_type_map();

        for (const auto &param_to_change : params)
        {
            // sets the drivebrain parameters
            _set_db_param(param_to_change);
        }
        // get the updated params
        auto fxglove_params_vec = _get_current_params();
        _server->publishParameterValues(clientHandle, fxglove_params_vec, request_id);
    };

    hdlrs.subscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle)
    {
        const auto clientStr = _server->remoteEndpointString(clientHandle);
        std::cout << "Client " << clientStr << " subscribed to " << chanId << std::endl;
    };

    hdlrs.unsubscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle)
    {
        const auto clientStr = _server->remoteEndpointString(clientHandle);
        std::cout << "Client " << clientStr << " unsubscribed from " << chanId << std::endl;
    };
    // TODO add all channels from the hytech_msgs.proto file https://chatgpt.com/share/66eb6d62-5ac0-8000-afb2-f2f0b00c1ede
    foxglove::ChannelWithoutId test_channel;
    test_channel.topic = hytech_msgs::MCUOutputData::descriptor()->name();
    hytech_msgs::MCUOutputData test;
    std::cout << hytech_msgs::MCUOutputData::descriptor()->name() << std::endl;
    std::cout << test.GetTypeName() << std::endl;

    test_channel.encoding = "protobuf";
    test_channel.schemaName = hytech_msgs::MCUOutputData::descriptor()->full_name();
    test_channel.schema = foxglove::base64Encode(SerializeFdSet(hytech_msgs::MCUOutputData::descriptor()));

    std::vector<foxglove::ChannelWithoutId> channels;
    channels.push_back(test_channel);
    auto res_ids = _server->addChannels(channels);

    std::vector<std::pair<std::string, foxglove::ChannelId>> zipped_channels;
    zipped_channels.reserve(channels.size()); // Reserve space to avoid reallocations

    std::transform(channels.begin(), channels.end(), res_ids.begin(),
                   std::back_inserter(zipped_channels),
                   [](foxglove::ChannelWithoutId a, foxglove::ChannelId b)
                   { return std::make_pair(a.topic, b); });

    for (const auto &id_map_pair : zipped_channels)
    {
        // first = channel def, second = id
        _id_name_map[id_map_pair.first] = id_map_pair.second;
    }
    _send_thread = std::thread(&core::FoxgloveWSServer::_handle_foxglove_send, this);

    _server->setHandlers(std::move(hdlrs));
    _server->start("0.0.0.0", 5555);
}

void core::FoxgloveWSServer::_handle_foxglove_send()
{
    // we will assume that this queue only has messages that we want to send
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> q;
    while (true)
    {
        {
            std::unique_lock lk(_out_queue.mtx);
            // TODO unfuck this, queue management shouldnt live within the queue itself
            _out_queue.cv.wait(lk, [this]()
                               { return !_out_queue.deque.empty(); });

            if (_out_queue.deque.empty())
            {
                return;
            }

            q.deque = _out_queue.deque;
            _out_queue.deque.clear();
        }

        for (const auto &msg : q.deque)
        {
            // auto can_msg = _get_msg(msg);
            // TODO check if the name exists within the map
            std::cout << "msg" << std::endl;
            auto msg_chan_id = _id_name_map[msg->GetDescriptor()->name()];
            const auto serializedMsg = msg->SerializeAsString();
            const auto now = nanosecondsSinceEpoch();
            _server->broadcastMessage(msg_chan_id, now, reinterpret_cast<const uint8_t *>(serializedMsg.data()),
                                      serializedMsg.size());
        }
        q.deque.clear();
    }
}

foxglove::Parameter core::FoxgloveWSServer::_get_foxglove_param(const std::string &set_name, core::common::Configurable::ParamTypes param)
{
    if (std::holds_alternative<bool>(param))
    {
        std::cout << set_name << " Variant holds an bool: " << std::get<bool>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<bool>(param));
    }
    else if (std::holds_alternative<int>(param))
    {
        std::cout << set_name << " Variant holds a int: " << std::get<int>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<int>(param));
    }
    else if (std::holds_alternative<float>(param))
    {
        std::cout << set_name << " Variant holds a float: " << std::get<float>(param) << std::endl;
        return foxglove::Parameter(set_name, ((double)std::get<float>(param)));
    }
    else if (std::holds_alternative<double>(param))
    {
        std::cout << set_name << " Variant holds a float: " << std::get<float>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<double>(param));
    }
    else if (std::holds_alternative<std::string>(param))
    {
        std::cout << set_name << " Variant holds a string: " << std::get<std::string>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<std::string>(param));
    }
    else
    {
        std::cout << set_name << " unknown param variant type" << std::endl;
    }

    return {};
}

core::common::Configurable::ParamTypes core::FoxgloveWSServer::_get_db_param(foxglove::Parameter param)
{

    if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_BOOL)
    {
        return param.getValue().getValue<bool>();
    }
    else if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_INTEGER)
    {
        return ((int)param.getValue().getValue<int64_t>());
    }
    else if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_DOUBLE)
    {
        return ((float)param.getValue().getValue<double>());
    }
    else if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_STRING)
    {
        return param.getValue().getValue<std::string>();
    }
    else
    {
        std::cout << "unsupported param type" << std::endl;
    }

    return std::monostate();
}

std::optional<foxglove::Parameter> core::FoxgloveWSServer::_convert_foxglove_param(core::common::Configurable::ParamTypes curr_param_val, foxglove::Parameter incoming_param)
{
    auto current_param_type = _get_foxglove_param(incoming_param.getName(), curr_param_val).getValue().getType();
    auto type = incoming_param.getValue().getType();

    using fpt = foxglove::ParameterType;
    if (_get_foxglove_param(incoming_param.getName(), curr_param_val).getValue().getType() == incoming_param.getValue().getType())
    {
        return incoming_param;
    }
    else if (current_param_type == fpt::PARAMETER_DOUBLE && type == fpt::PARAMETER_INTEGER)
    {
        return foxglove::Parameter(incoming_param.getName(), static_cast<double>(incoming_param.getValue().getValue<int64_t>()));
    }
    else if (current_param_type == fpt::PARAMETER_INTEGER && type == fpt::PARAMETER_DOUBLE)
    {
        return foxglove::Parameter(incoming_param.getName(), static_cast<int64_t>(incoming_param.getValue().getValue<double>()));
    }
    else
    {
        std::cout << "WARNING: unsupported type input, not setting" << std::endl;
        return std::nullopt;
    }
}

void core::FoxgloveWSServer::_set_db_param(foxglove::Parameter param_update)
{
    auto split_pos = param_update.getName().find("/");

    auto param_name = param_update.getName().substr(split_pos + 1);
    auto component_name = param_update.getName().substr(0, split_pos);

    for (const auto component : _components)
    {
        if (component->get_name() == component_name)
        {

            auto curr_param_val = component->get_cached_param(param_name);
            auto converted_type = _convert_foxglove_param(curr_param_val, param_update);
            if (converted_type)
            {
                auto val = _get_db_param(*converted_type);
                component->handle_live_param_update(param_name, val);
            }
            return;
        }
    }
    std::cout << "WARNING: could not find component " << component_name << std::endl;
}

std::vector<foxglove::Parameter> core::FoxgloveWSServer::_get_current_params()
{
    std::vector<foxglove::Parameter> params;
    for (const auto component : _components)
    {
        auto params_map = component->get_params_map();
        auto param_parent = component->get_name();
        auto param_names = component->get_param_names();
        for (const auto &component_param_name : param_names)
        {
            auto foxglove_param_id = param_parent + "/" + component_param_name;
            auto fxglove_param = _get_foxglove_param(foxglove_param_id, params_map[component_param_name]);
            params.push_back(fxglove_param);
        }
    }
    return params;
}
