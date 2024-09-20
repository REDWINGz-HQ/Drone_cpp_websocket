#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <optional>
#include <vector>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/info/info.h>
#include <mavsdk/plugins/mission/mission.h>
#include <typeinfo>
#include <sstream>
#include <cmath>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;
using json = nlohmann::json;

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

// Function to get mission data from WebSocket
std::string get_data_from_websocket(const std::string& hardware_uid) {
    try {
        net::io_context ioc;
        tcp::resolver resolver{ioc};
        websocket::stream<tcp::socket> ws{ioc};

        // Resolve and connect 
        // 34.143.225.243
        auto const results = resolver.resolve("34.143.225.243", "8000");
        net::connect(ws.next_layer(), results.begin(), results.end());

        // WebSocket handshake and send client name
        ws.handshake("34.143.225.243", "/");
        ws.write(net::buffer(hardware_uid));

        // Read a message into our buffer
        beast::flat_buffer buffer;
        ws.read(buffer);

        // Close the WebSocket connection
        ws.close(websocket::close_code::normal);

        // Return the data as a string
        return beast::buffers_to_string(buffer.data());
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return "";
    }
}

float home_alt_check(Telemetry &telemetry){
    Telemetry::Position home_position = telemetry.position();
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << "Home Lat: " << home_position.latitude_deg << std::endl;
    std::cout << "Home Lon: " << home_position.longitude_deg << std::endl;
    std::cout << "Home Alt: " << home_position.absolute_altitude_m << " meters" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;

    return home_position.absolute_altitude_m;
}

bool drone_arming(Action &action){
    const Action::Result arm_result = action.arm();

    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << std::endl;
        return false;
    }
    return true;
}

bool drone_disarming(Action &action){
    const Action::Result disarm_result = action.disarm();

    if (disarm_result != Action::Result::Success) {
        std::cerr << "Disarming failed: " << disarm_result << std::endl;
        return false;
    }
    return true;
}

bool drone_takeoff(Action &action, const float& tkf_alt){
    // float tkf_alt = j["param3"].get<float>();     //alt(22) above ground

    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();

    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << std::endl;
        return false;
    }

    const Action::Result set_takeoff_result = action.set_takeoff_altitude(tkf_alt);
    if (set_takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << set_takeoff_result << std::endl;
        return false;
    }

    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << std::endl;
        return false;
    }
    return true;
}

bool drone_landing(Action &action){
    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "Land failed: " << land_result << std::endl;
        return false;
    }
    return true;
}

bool drone_rtl(Action &action){
    const Action::Result rtl_result = action.return_to_launch();
    if (rtl_result != Action::Result::Success) {
        std::cerr << "RTL failed: " << rtl_result << std::endl;
        return false;
    }
    return true;
}

bool drone_hold(Action &action){
    const Action::Result hold_result = action.hold();
    if (hold_result != Action::Result::Success) {
        std::cerr << "Hold failed: " << hold_result << std::endl;
        return false;
    }
    return true;
}

bool drone_start_mission(Mission &mission){
    const Mission::Result start_mis_result = mission.start_mission();
    if (start_mis_result != Mission::Result::Success) {
        std::cerr << "Start mission failed: " << start_mis_result << std::endl;
        return false;
    }
    return true;
}

bool drone_pause_mission(Mission &mission){
    const Mission::Result pause_mis_result = mission.pause_mission();
    if (pause_mis_result != Mission::Result::Success) {
        std::cerr << "Pause mission failed: " << pause_mis_result << std::endl;
        return false;
    }
    return true;
}

bool drone_gtlo(Action &action, const float& home_alt_amsl, const double& lat, 
                const double& lon, const float& alt, const float& yaw){
    // double lat = j["lat"].get<double>();     //lat(16)
    // double lon = j["lon"].get<double>();     //lon(16)
    // float alt = j["alt"].get<float>();     //alt(16) 
    // float yaw = j["yaw"].get<float>();     //yaw(16)
    
    // Telemetry::Position current_position = telemetry.position();

    // double current_lat = current_position.latitude_deg;
    // double current_lon = current_position.longitude_deg;
    // double yaw_auto = calculate_bearing(current_lat, current_lon, lat, lon);

    float alt_amsl = home_alt_amsl + alt;   //change amsl into above ground
    
    const Action::Result gtlo_result = action.goto_location(lat, lon, alt_amsl, yaw);
    std::cout << "Go to next location (lat, lon, alt) " << lat << ", " << lon << ", " << alt << std::endl;
    if (gtlo_result != Action::Result::Success) {
        std::cerr << "Go to location failed: " << gtlo_result << std::endl;
        return false;
    }
    return true;
}

bool drone_orbit(Action &action, const float& home_alt_amsl, const float& orbit_r,
                 const float& orbit_v, const int& yaw_behav, const double& lat,
                 const double& lon, const float& alt){
    // float orbit_r = j["orbit_r"].get<float>();
    // float orbit_v = j["orbit_v"].get<float>();
    // int yaw_behav = j["yaw_behav"].get<int>();
    // double lat = j["lat"].get<double>();
    // double lon = j["lon"].get<double>();
    // double alt = j["alt"].get<double>();        //alt amsl

    mavsdk::Action::OrbitYawBehavior yaw_case;

    switch (yaw_behav){
        case 1:
            yaw_case = mavsdk::Action::OrbitYawBehavior::HoldFrontToCircleCenter;       //Vehicle front points to the center (default).
            break;
        case 2:
            yaw_case = mavsdk::Action::OrbitYawBehavior::HoldInitialHeading;            //Vehicle front holds heading when message received.
            break;
        case 3:
            yaw_case = mavsdk::Action::OrbitYawBehavior::Uncontrolled;                  //Yaw uncontrolled.
            break;
        case 4:
            yaw_case = mavsdk::Action::OrbitYawBehavior::HoldFrontTangentToCircle;      //Vehicle front follows flight path (tangential to circle).
            break;
        case 5:
            yaw_case = mavsdk::Action::OrbitYawBehavior::RcControlled;                  //Yaw controlled by RC input.
            break;
        default:
            std::cout << "Invalid option" << std::endl;
            break;
    }

    double alt_amsl = home_alt_amsl + alt;
    const Action::Result orbit_result = action.do_orbit(orbit_r, orbit_v, yaw_case, lat, lon, alt_amsl);    //alt above sea level
    std::cout << "Orbit with " << yaw_case << std::endl;

    if (orbit_result != Action::Result::Success) {
        std::cerr << "Go to location failed: " << orbit_result << std::endl;
        return false;
    }
    return true;
}

bool drone_trans_to_fw(Action &action){
    const Action::Result trans_to_fw_res = action.transition_to_fixedwing();
    if (trans_to_fw_res != Action::Result::Success) {
        std::cerr << "Transition to fw failed: " << trans_to_fw_res << std::endl;
        return false;
    }
    return true;
}

bool drone_trans_to_mc(Action &action){
    const Action::Result trans_to_mc_res = action.transition_to_multicopter();
    if (trans_to_mc_res != Action::Result::Success) {
        std::cerr << "Transition to mc failed: " << trans_to_mc_res << std::endl;
        return false;
    }
    return true;
}

void drone_action(Action &action, Mission &mission, const std::string& action_data, const float& home_alt_amsl){

    json j = json::parse(action_data);
    std::string action_type;

    uint16_t command = j["command"].get<uint16_t>();
    double param1 = j["param1"].get<double>();      //lat(16),        , lat(34),     
    double param2 = j["param2"].get<double>();      //lon(16),        , lon(34)
    float param3 = j["param3"].get<float>();        //alt(16), alt(22), alt(34)
    float param4 = j["param4"].get<float>();        //yaw(16),        , orbit_r(34)
    float param5 = j["param5"].get<float>();        //                , orbit_v(34)
    int param6 = j["param6"].get<int>();            //                , yaw_behav(34)

    // double param1 = std::stod(j["param1"].get<std::string>()); //lat
    // double param2 = std::stod(j["param2"].get<std::string>()); //lon
    
    switch (command) {
            case 1:
                action_type = "Arm";
                // drone_arming(action);
                if (!drone_arming(action)){
                    std::cerr << "Failed to perform Arming action" << std::endl;
                }
                break;
            case 2:
                action_type = "Disarm";
                // drone_disarming(action);
                if (!drone_disarming(action)){
                    std::cerr << "Failed to perform Disarming action" << std::endl;
                }
                break;
            case 16:
                action_type = "Go to location";
                if (!drone_gtlo(action, home_alt_amsl, param1, param2, param3, param4)){ //alt amsl
                    std::cerr << "Failed to perform Go to location action" << std::endl;
                }
                break;
            case 17:
                action_type = "Hold";
                if (!drone_hold(action)){
                    std::cerr << "Failed to perform Hold action" << std::endl;
                }
                break;
            case 20:
                action_type = "RTL";
                if (!drone_rtl(action)){
                    std::cerr << "Failed to perform RTL action" << std::endl;
                }
                break;
            case 21:
                action_type = "Landing";
                if (!drone_landing(action)){
                    std::cerr << "Failed to perform Landing action" << std::endl;
                }
                break;
            case 22:
                action_type = "Takeoff";
                if (!drone_takeoff(action, param3)){
                    std::cerr << "Failed to perform Takeoff action" << std::endl;
                }
                break;
            case 34:
                action_type = "Orbit";
                if (!drone_orbit(action, home_alt_amsl, param4, param5, param6, param1, param2, param3)){
                    std::cerr << "Failed to perform Orbit action" << std::endl;
                }
                break;
            case 300:
                action_type = "Start mission";
                if (!drone_start_mission(mission)){
                    std::cerr << "Failed to perform Start mission action" << std::endl;
                }
                break;
            case 301:
                action_type = "Stop mission";
                if (!drone_pause_mission(mission)){
                    std::cerr << "Failed to perform Stop mission action" << std::endl;
                }
                break;
            case 3000:
                action_type = "Transition to fw";
                if (!drone_trans_to_fw(action)){
                    std::cerr << "Failed to perform Transition to fw action" << std::endl;
                }
                break;
            case 3001:
                action_type = "Transition to mc";
                if (!drone_trans_to_mc(action)){
                    std::cerr << "Failed to perform Transition to mc action" << std::endl;
                }
                break;
            default:
                std::cout << "Invalid option" << std::endl;
                break;
        }
    
    std::cout << "Received: " << action_type << " Command" << std::endl;
    // std::cout << "Param1: " << param1 << " Type: " << typeid(param1).name() << std::endl;
    // std::cout << "Param2: " << param2 << " Type: " << typeid(param2).name() << std::endl;
    // std::cout << "Param3: " << param3 << " Type: " << typeid(param3).name() << std::endl;
    // std::cout << "Param4: " << param4 << " Type: " << typeid(param4).name() << std::endl;

}

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <connection_url>" << std::endl;
        return 1;
    }

    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << std::endl;
        return 1;
    }

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system" << std::endl;
        return 1;
    }

    auto action = Action{system.value()};
    auto telemetry = Telemetry{system.value()};
    auto info = Info{system.value()};
    auto mission = Mission{system.value()};
    // auto mavpassthrough = MavlinkPassthrough{system.value()};

    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready\n";
        sleep_for(seconds(1));
    }

    //check home pos
    Telemetry::Health home_health = telemetry.health();
    while (!home_health.is_home_position_ok){
        std::cout << "Home position is not ok" << std::endl;
        sleep_for(seconds(1));
    }
    sleep_for(seconds(5));
    float home_alt_amsl = home_alt_check(telemetry);


	auto info_pair = info.get_identification();
	mavsdk::Info::Identification unique_info = info_pair.second;
	std::string hardware_iden_num = unique_info.hardware_uid;

    // for (size_t i = 0; i < hardware_iden_num.length(); ++i) {
    //     std::cout << "Char: " << hardware_iden_num[i] 
    //             << " ASCII: " << static_cast<int>(hardware_iden_num[i]) 
    //             << std::endl;
    // }

    if (hardware_iden_num.back() == '\0') {
        hardware_iden_num.pop_back();
    }

    // std::cout << "uid from info pair : " << hardware_iden_num << " Type: "<< hardware_iden_num.length() << std::endl;

    std::cout << "System ready" << std::endl;
    std::cout << "Receiving mission data from WebSocket..." << std::endl;

    for (;;) {
        std::string action_data = get_data_from_websocket(hardware_iden_num); // here
        if (action_data.empty()) {
            std::cerr << "Failed to receive data from WebSocket\n";
            return 1;
        }

        std::cout << "=====================================" << std::endl;
        drone_action(action, mission, action_data, home_alt_amsl);

        // if (result != MavlinkPassthrough::Result::Success) {
        //     std::cerr << "Failed to send command: " << mavpassthrough.result_str(result) << std::endl;
        // }
    }

}