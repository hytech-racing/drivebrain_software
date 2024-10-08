#include <db_service/v1/service/db_interface.grpc.pb.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>

class DBInterfaceImpl final : public db_service::v1::service::DBInterface::Service {
    grpc::Status RequestStopLogging(grpc::ServerContext* context, const google::protobuf::Empty *rq, db_service::v1::service::LoggerStatus * response)
    override {
        std::cout << "requested stopping of logging" <<std::endl;
        response->set_currently_logging(false);
        response->set_active_or_previous_log_file_name("dummyname");
        return grpc::Status::OK;
    }
};

void RunServer() {
  std::string server_address = "0.0.0.0:6969";
  DBInterfaceImpl service;

  grpc::reflection::InitProtoReflectionServerBuilderPlugin();
  grpc::ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  builder.RegisterService(&service);
  // Finally assemble the server.
  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;

  // Wait for the server to shutdown. Note that some other thread must be
  // responsible for shutting down the server for this call to ever return.
  server->Wait();
}

int main(int argc, char** argv) {
  RunServer();
  return 0;
}