#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "hytech_msgs.pb.h"
#include <thread>

void sendACUTestData(const std::string &ip, int port)
{
    // Create a UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        std::cerr << "Failed to create socket" << std::endl;
        return;
    }

    // Set up the destination address
    sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &dest_addr.sin_addr);

    // Create and populate the SpeedControlIn protobuf message
    hytech_msgs::ACUAllData acu_data;

    float accl = 1.0;
    while (true)
    {
        accl += 0.0001;
        acu_data.mutable_core_data()->set_max_cell_temp(accl); 
        

        
        // Serialize the message to a string
        std::string serialized_message;
        if (!acu_data.SerializeToString(&serialized_message))
        {
            std::cerr << "Failed to serialize the message" << std::endl;
            close(sock);
            return;
        }

        // Send the serialized message over UDP
        ssize_t sent_bytes = sendto(sock, serialized_message.c_str(), serialized_message.size(), 0,
                                    (struct sockaddr *)&dest_addr, sizeof(dest_addr));

        if (sent_bytes < 0)
        {
            std::cerr << "Failed to send message" << std::endl;
        }
        else
        {
            std::cout << "Sent " << sent_bytes << " bytes to " << ip << ":" << port << std::endl;
        }

        using namespace std::chrono_literals;
 

 
        std::this_thread::sleep_for(1ms);
    }

    // Close the socket
    close(sock);
}

int main()
{
    // Configuration
    std::string target_ip = "localhost"; // IP address to send to
    int target_port = 7766;             // Port number to send to

    // Send the SpeedControlIn message
    sendACUTestData(target_ip, target_port);

    return 0;
}
