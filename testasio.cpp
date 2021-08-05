#include <asio.hpp>
#include <string>
#include <thread>
#include <chrono>
#include <msgpack.hpp>

using namespace asio;

//
// Send a string via UDP to the specified destination
// ip addresss at the specified port (point-to-point
// not broadcast)
//
bool send_udp_message(const std::string& message, const std::string& destination_ip,
	const unsigned short port) {
	io_service io_service;
	ip::udp::socket socket(io_service);

	// Create the remote endpoint using the destination ip address and
	// the target port number.  This is not a broadcast
	auto remote = ip::udp::endpoint(ip::address::from_string(destination_ip), port);

	try {

		// Open the socket, socket's destructor will
		// automatically close it.
		socket.open(asio::ip::udp::v4());

		// And send the string... (synchronous / blocking)
		socket.send_to(buffer(message), remote);

	}
	catch (const asio::system_error& ex) {
		// Exception thrown!
		// Examine ex.code() and ex.what() to see what went wrong!
		return false;
	}

	return true;
}

int main() {
	std::string destination_ip = "127.0.0.1";
	unsigned short port = 32000;

	io_service io_service;
	ip::udp::socket socket(io_service);

	// Create the remote endpoint using the destination ip address and
	// the target port number.  This is not a broadcast
	auto remote = ip::udp::endpoint(ip::address::from_string(destination_ip), port);

	// Open the socket, socket's destructor will
	// automatically close it.
	socket.open(asio::ip::udp::v4());

	std::string message = "abcde";
	while (1) {

		msgpack::sbuffer sb;
		msgpack::pack(sb, std::make_tuple(42, false, "hello world", 12.3456));

		//send_udp_message("abcde", "127.0.0.1", 32000);
		// And send the string... (synchronous / blocking)
		socket.send_to(buffer(sb.data(),sb.size()), remote);
		std::this_thread::sleep_for(std::chrono::milliseconds(150));
		

	}


	return 0;
}