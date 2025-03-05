#include <Controller.hpp>
#include <Configurable.hpp>
#include <Logger.hpp>
#include <chrono>
#include <hytech_msgs.pb.h>
#include <Literals.hpp>
#include <hytech.pb.h>
#include <VehicleDataTypes.hpp>

#include <DrivebrainMCAPLogger.hpp>

#include <MsgLogger.hpp>
#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <thread>
#include <type_traits>
#include <utility>
#include <mutex>
#include <variant>

// ABOUT: this controller is an implementation of mode 0

// this controller will be reactionary for now
// namespace control
// {

    // TODO make the output CAN message for the drivetrain, rpms telem is just a standin for now
class ConfigureableTest : public core::common::Configurable
{
public:
    ConfigureableTest(core::Logger &logger, core::JsonFileHandler &json_file_handler) : _logger(logger), Configurable(_logger, json_file_handler, "ConfigureableTest") {}
    bool init() override {
        std::optional test_float = get_parameter_value<float>("test_float");
        std::optional test_bool = get_parameter_value<bool>("test_bool");
        std::optional test_double = get_parameter_value<double>("test_double");
        std::optional test_string = get_parameter_value<std::string>("test_string");
        std::optional test_int = get_parameter_value<int>("test_int");
        // _configured = true;
        set_configured();
        return true;
    }
private:
    core::Logger &_logger;
};

class ConfigureableTest2 : public core::common::Configurable
{
public:
    ConfigureableTest2(core::Logger &logger, core::JsonFileHandler &json_file_handler) : _logger(logger), Configurable(_logger, json_file_handler, "asdf2") {}
    bool init() override {
        std::optional test_float = get_parameter_value<float>("test_val");
        // _configured = true;
        set_configured();
        return true;
    }
private:
    core::Logger &_logger;
};

int main()
{
    // TODO:

    // - [x] test new configureable instantiation to ensure that schema gets incrementally created as new params are "gotten"
    // - [ ] test writing of the schema and creation of the config topic in the mcap file that gets created
    // - [ ] test the writing of the config json message containing all the config data from every component
    core::JsonFileHandler config("../config/config_test.json");
    core::Logger logger(core::LogLevel::INFO);

    auto test = std::make_shared<ConfigureableTest>(logger, config);
    auto test_inst = std::make_shared<ConfigureableTest2>(logger, config);
    
    

    std::vector<std::shared_ptr<core::common::Configurable>> configureable_components;
    auto test_inst_cast = std::reinterpret_pointer_cast<core::common::Configurable>(test);
    configureable_components.push_back(test_inst_cast);
    configureable_components.push_back(std::reinterpret_pointer_cast<core::common::Configurable>(test_inst));

    auto standin_foxglove_ws_send = [](std::shared_ptr<google::protobuf::Message> msg){};
    auto mcap_logger = std::make_shared<common::DrivebrainMCAPLogger>("temp", configureable_components);

    test->init(); // everything has to be initialized BEFORE the message logger is created.
    test_inst->init();
    auto msg_logger = std::make_unique<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>>(".mcap", true, 
        std::bind(&common::DrivebrainMCAPLogger::log_msg, mcap_logger, std::placeholders::_1),
        std::bind(&common::DrivebrainMCAPLogger::close_current_mcap, mcap_logger),
        std::bind(&common::DrivebrainMCAPLogger::open_new_mcap, mcap_logger, std::placeholders::_1),
        standin_foxglove_ws_send,
        std::bind(&common::DrivebrainMCAPLogger::init_param_schema, mcap_logger),
        std::bind(&common::DrivebrainMCAPLogger::log_params, mcap_logger));
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::seconds(2)));
    
    // std::cout << test.get_schema().dump() << std::endl;
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::seconds(10)));
    return 0;
}