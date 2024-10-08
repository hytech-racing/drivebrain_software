#include <db_service/v1/service/db_interface.grpc.pb.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>

#include <MsgLogger.hpp>

class DBInterfaceImpl final : public db_service::v1::service::DBInterface::Service {

    struct InitStruct {
        std::shared_ptr<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>> logger_inst;
    };
    grpc::Status RequestStopLogging(grpc::ServerContext* context, const google::protobuf::Empty *rq, db_service::v1::service::LoggerStatus * response) override; 
    grpc::Status RequestStartLogging(grpc::ServerContext* context, const google::protobuf::Empty *rq, db_service::v1::service::LoggerStatus * response) override;
    grpc::Status RequestCurrentLoggerStatus(grpc::ServerContext* context, const google::protobuf::Empty* request, db_service::v1::service::LoggerStatus* response) override; 
    
    public: 
        DBInterfaceImpl(std::shared_ptr<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>> logger_inst);
        void run_server(); 
        void stop_server();
    private:
        std::shared_ptr<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>> _logger_inst;
        std::unique_ptr<grpc::Server> _server;  // Store server instance here

};