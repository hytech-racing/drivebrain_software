#include <foxglove_server.hpp>
#include <variant>
#include <hytech_msgs.pb.h>
#include <ProtobufUtils.hpp>

#include <queue>
#include <unordered_map>

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

core::FoxgloveWSServer::FoxgloveWSServer(std::vector<core::common::Configurable *> configurable_components) : _components(configurable_components)
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

    // TODO make the .proto file name a parameter

    auto potential_id_map = util::generate_name_to_id_map({"hytech_msgs.proto", "hytech.proto"});
    if (potential_id_map)
    {
        _id_name_map = *potential_id_map;
    }

    auto descriptors = util::get_pb_descriptors({"hytech_msgs.proto", "hytech.proto"});

    std::vector<foxglove::ChannelWithoutId> channels;

    for (const auto &file_descriptor : descriptors)
    {

        for (int i = 0; i < file_descriptor->message_type_count(); ++i)
        {
            const google::protobuf::Descriptor *message_descriptor = file_descriptor->message_type(i);
            foxglove::ChannelWithoutId server_channel;
            server_channel.topic = message_descriptor->name();
            server_channel.encoding = "protobuf";
            server_channel.schemaName = message_descriptor->full_name();
            server_channel.schema = foxglove::base64Encode(SerializeFdSet(message_descriptor));
            channels.push_back(server_channel);
        }
    }

    auto res_ids = _server->addChannels(channels);
    _server->setHandlers(std::move(hdlrs));
    _server->start("0.0.0.0", 5555);
}

void core::FoxgloveWSServer::send_live_telem_msg(std::shared_ptr<google::protobuf::Message> msg)
{

    if (_id_name_map.find(msg->GetDescriptor()->name()) != _id_name_map.end())
    {
        auto msg_chan_id = _id_name_map[msg->GetDescriptor()->name()];
        const auto serializedMsg = msg->SerializeAsString();
        const auto now = nanosecondsSinceEpoch();
        _server->broadcastMessage(msg_chan_id, now, reinterpret_cast<const uint8_t *>(serializedMsg.data()),
                                  serializedMsg.size());
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
        // std::cout << set_name << " Variant holds a int: " << std::get<int>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<int>(param));
    }
    else if (std::holds_alternative<float>(param))
    {
        // std::cout << set_name << " Variant holds a float: " << std::get<float>(param) << std::endl;
        return foxglove::Parameter(set_name, ((double)std::get<float>(param)));
    }
    else if (std::holds_alternative<double>(param))
    {
        // std::cout << set_name << " Variant holds a float: " << std::get<float>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<double>(param));
    }
    else if (std::holds_alternative<std::string>(param))
    {
        // std::cout << set_name << " Variant holds a string: " << std::get<std::string>(param) << std::endl;
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
