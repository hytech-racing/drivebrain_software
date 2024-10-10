#include <DBServiceImpl.hpp>

grpc::Status DBInterfaceImpl::RequestStopLogging(grpc::ServerContext *context, const google::protobuf::Empty *rq, db_service::v1::service::LoggerStatus *response)
{
    {
        std::cout << "requested stopping of logging" << std::endl;
        _logger_inst->stop_logging_to_file();
        auto status = _logger_inst->get_logger_status();

        response->set_active_or_previous_log_file_name(std::get<0>(status));
        response->set_currently_logging(std::get<1>(status));
        return grpc::Status::OK;
    }
}
grpc::Status DBInterfaceImpl::RequestStartLogging(grpc::ServerContext *context, const google::protobuf::Empty *rq, db_service::v1::service::LoggerStatus *response)
{
    {
        std::cout << "requested start of logging" << std::endl;
        _logger_inst->start_logging_to_new_file();
        auto status = _logger_inst->get_logger_status();
        response->set_active_or_previous_log_file_name(std::get<0>(status));
        response->set_currently_logging(std::get<1>(status));
        return grpc::Status::OK;
    }
}
grpc::Status DBInterfaceImpl::RequestCurrentLoggerStatus(grpc::ServerContext *context, const google::protobuf::Empty *rq, db_service::v1::service::LoggerStatus *response)
{
    {
        std::cout << "requested status of logger" << std::endl;
        auto status = _logger_inst->get_logger_status();
        response->set_active_or_previous_log_file_name(std::get<0>(status));
        response->set_currently_logging(std::get<1>(status));
        return grpc::Status::OK;
    }
}

grpc::Status DBInterfaceImpl::RequestCurrentVectornavRotMatrix(grpc::ServerContext *context, const google::protobuf::Empty *rq, db_service::v1::service::LoggerStatus *response)
{
    _vn_driver->
}

DBInterfaceImpl::DBInterfaceImpl(std::shared_ptr<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>> logger_inst,
                                 VNDriver *vn_driver) : _logger_inst(logger_inst),
                                                        _vn_driver(vn_driver)
{
}
void DBInterfaceImpl::stop_server()
{
    if (_server)
    {
        std::cout << "Shutting down the server..." << std::endl;
        _server->Shutdown();
    }
}
void DBInterfaceImpl::run_server()
{
    std::string server_address = "0.0.0.0:6969";

    grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    grpc::ServerBuilder builder;
    // Listen on the given address without any authentication mechanism.
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Register "service" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *synchronous* service.
    builder.RegisterService(this);
    // Finally assemble the server.
    _server = builder.BuildAndStart();
    std::cout << "Server listening on " << server_address << std::endl;

    // Wait for the server to shutdown. Note that some other thread must be
    // responsible for shutting down the server for this call to ever return.
    _server->Wait();
}
