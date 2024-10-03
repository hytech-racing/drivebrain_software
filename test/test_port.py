import socket

def send_udp_message(message: str, ip: str, port: int):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Send the message to the specified IP and port
    sock.sendto(message.encode(), (ip, port))

    # Close the socket
    sock.close()

if __name__ == "__main__":
    # Configurable IP and port
    target_ip = "127.0.0.1"  # Localhost for testing
    target_port = 12411        # Configurable port

    # Message to send
    message = "Hello, UDP!"

    # Send the message
    send_udp_message(message, target_ip, target_port)
