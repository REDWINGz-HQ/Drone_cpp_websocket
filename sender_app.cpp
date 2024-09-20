#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <iostream>
#include <fstream> // For file reading
#include <nlohmann/json.hpp>  // JSON parsing
#include <thread>  // For sleep
#include <chrono>  // For std::chrono::seconds

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;
using json = nlohmann::json;

int main() {
    try {
        // Create io_context and WebSocket stream
        net::io_context ioc;
        tcp::resolver resolver(ioc);
        websocket::stream<tcp::socket> ws(ioc);

        // Resolve and connect to the WebSocket server
        auto const results = resolver.resolve("34.143.225.243", "8000");
        auto ep = net::connect(ws.next_layer(), results);

        // Perform WebSocket handshake
        ws.handshake("34.143.225.243", "/");

        // Send client identifier (e.g., cpp_app)
        ws.write(net::buffer(std::string("cpp_app")));

        // Read the JSON file
        std::ifstream file("data.json");
        if (!file.is_open()) {
            std::cerr << "Error: Could not open the JSON file." << std::endl;
            return 1;
        }

        // Parse the JSON file
        json j;
        file >> j;
        file.close();  // Close the file after reading

        // Convert JSON object to string
        std::string message01 = j.dump();  // Dump the JSON as a string

        // Assign target_client
        j["target_client"] = "000000000000000000000000000000000000";
        // std::string message01 = "returntolaunch";
        std::string message02 = "this msg for client 001234";

        // Optionally, you can send a message to a specific client
        std::string target_client01 = j["target_client"];
        // std::string target_client01 = "000000000000000000000000000000000000";
        // std::cout << "uid from fixed client: " << target_client << " Type: "<< target_client.length() << std::endl;

        std::cout << target_client01 << ":" << message01 << std::endl;
        std::cout << "----------------------------------------------" << std::endl;

        ws.write(net::buffer(target_client01 + ":" + message01));
        // Close the WebSocket connection gracefully
        ws.close(websocket::close_code::normal);
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}