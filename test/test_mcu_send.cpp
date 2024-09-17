#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "hytech_msgs.pb.h"
void sendSpeedControlInMessage(const std::string &ip, int port)
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
    hytech_msgs::MCUCommandData speed_control_out;

    float accl = 1.0;
    while (true)
    {
        accl += 0.0001;
        auto des_rpms = speed_control_out.mutable_desired_rpms();  
        des_rpms->set_fl(accl);
        des_rpms->set_fr(accl);
        des_rpms->set_rl(accl);
        des_rpms->set_rr(accl);


        auto tq_pos_lim = speed_control_out.mutable_torque_limit_nm(); 
        tq_pos_lim->set_fl(6969.0);
        tq_pos_lim->set_fr(6969.0);
        tq_pos_lim->set_rl(6969.0);
        tq_pos_lim->set_rr(6969.0);
        
        // Serialize the message to a string
        std::string serialized_message;
        if (!speed_control_out.SerializeToString(&serialized_message))
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
    }

    // Close the socket
    close(sock);
}

int main()
{
    // Configuration
    std::string target_ip = "192.168.1.30"; // IP address to send to
    int target_port = 2000;             // Port number to send to

    // Send the SpeedControlIn message
    sendSpeedControlInMessage(target_ip, target_port);

    return 0;
}
