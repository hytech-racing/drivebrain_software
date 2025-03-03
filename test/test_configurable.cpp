#include <Controller.hpp>
#include <Configurable.hpp>
#include <Logger.hpp>
#include <hytech_msgs.pb.h>
#include <Literals.hpp>
#include <hytech.pb.h>
#include <VehicleDataTypes.hpp>
#include <utility>
#include <mutex>

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
            return true;
        }
    private:
        core::Logger &_logger;
    };


int main()
{
    core::JsonFileHandler config("../config/config_test.json");
    core::Logger logger(core::LogLevel::INFO);

    ConfigureableTest test(logger, config);
    test.init();
    std::cout << test.get_schema().dump() << std::endl;

    return 0;
}