#include <arpa/inet.h>
#include <libkern/OSByteOrder.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <cstring>
#include <iostream>

bool runloop = true;
void sighandler(int) { runloop = false; }

// Function to convert a big-endian byte array to a double
double decodeToDouble(const unsigned char* buffer) {
	double value;
	std::memcpy(&value, buffer, sizeof(value));

	// Check system endianness
	if (OSHostByteOrder() == OSLittleEndian) {
		// If little-endian, reverse the byte order
		unsigned char temp_buffer[8];
		for (int i = 0; i < 8; ++i) {
			temp_buffer[i] = buffer[7 - i];
		}
		std::memcpy(&value, temp_buffer, sizeof(value));
	}

	return value;
}

int main() {
	// signal handling
	signal(SIGINT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGABRT, &sighandler);

	// UDP socket setup
	int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0) {
		std::cerr << "Socket creation failed" << std::endl;
		return -1;
	}

    // Set the socket to non-blocking mode
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags < 0) {
        std::cerr << "Failed to get socket flags" << std::endl;
        close(sockfd);
        return -1;
    }
    if (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) < 0) {
        std::cerr << "Failed to set socket to non-blocking" << std::endl;
        close(sockfd);
        return -1;
    }

	// Bind the socket to a port
	struct sockaddr_in server_addr, client_addr;
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(8080);
	server_addr.sin_addr.s_addr = INADDR_ANY;

	if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
		std::cerr << "Binding failed" << std::endl;
		close(sockfd);
		return -1;
	}

	// Buffer to store the incoming data (3 doubles = 24 bytes)
	unsigned char buffer[24];
	socklen_t client_len = sizeof(client_addr);

	while (runloop) {

		usleep(10000);

		// Receive data
		ssize_t received_bytes =
			recvfrom(sockfd, buffer, sizeof(buffer), 0,
					 (struct sockaddr*)&client_addr, &client_len);

        if (received_bytes < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                // No data available, non-blocking mode
                // std::cout << "No data available, continue..." << std::endl;
                // usleep(100000);  // Sleep for 100ms before checking again
                continue;
            } else {
                std::cerr << "Error in recvfrom(): " << strerror(errno) << std::endl;
                break;
            }
        }

		if (received_bytes != sizeof(buffer)) {
			std::cerr << "Failed to receive all bytes" << std::endl;
		} else {
			std::cout << "\nMessage received successfully!" << std::endl;

			// Convert the received bytes to doubles
			double value1 = decodeToDouble(buffer);
			double value2 = decodeToDouble(buffer + 8);
			double value3 = decodeToDouble(buffer + 16);

			// Print the received values
			std::cout << "Received values: " << std::endl;
			std::cout << "Value 1: " << value1 << std::endl;
			std::cout << "Value 2: " << value2 << std::endl;
			std::cout << "Value 3: " << value3 << std::endl;
		}
	}

	// Close the socket
	close(sockfd);

	return 0;
}
