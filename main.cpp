//HEADLESS=0 make px4_sitl jmavsim
//cd Documents/Johnnette_tech/GCS_cpp/Firmware
// mkdir build && cd build
// cmake ..
// make
// -------OR----------
// cmake CMakeLists.txt
// make all or make cmake_hello
// ./GCS udp://:14540
// #include <mavsdk/mavsdk.h>
// #include <mavsdk/plugins/action/action.h>
// #include <mavsdk/plugins/telemetry/telemetry.h>
// #include <iostream>
// #include <thread>
// #include <chrono>
// #include <cstdint>
// #include <future>
// int main(){
// mavsdk::Mavsdk mavsdk1;
// std::string connection_url="udp://:14540";
// mavsdk::ConnectionResult connection_result = mavsdk1.add_any_connection(connection_url);
// //ASSERT_EQ(connection_result, mavsdk::ConnectionResult::Success)
// if (connection_result != mavsdk::ConnectionResult::Success) {
//         std::cout <<"Connection failed: " << connection_result << std::endl;
//         return 1;
//     }
// auto new_system_promise = std::promise<std::shared_ptr<mavsdk::System>>{};
// auto new_system_future = new_system_promise.get_future();
// mavsdk1.subscribe_on_new_system([&mavsdk1, &new_system_promise]() {
//     new_system_promise.set_value(mavsdk1.systems().at(0));
//     mavsdk1.subscribe_on_new_system(nullptr);
//     });
// // while (mavsdk1.systems().size() == 0) {
// //     std::this_thread::sleep_for(std::chrono::seconds(1));
// // }
// auto system = new_system_future.get();
// system->subscribe_is_connected([](bool is_connected) {
//     if (is_connected) {
//         std::cout << "System has been discovered" << '\n';
//     } else {
//         std::cout << "System has timed out" << '\n';
//     }
// });
// return 0;
// }


#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/log_files/log_files.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <functional>
#include <iostream>
#include <thread>
#include <future>

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;
using namespace std::placeholders; // for `_1`


#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

static Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    Mission::MissionItem::CameraAction camera_action);
int log_entries(LogFiles& log_files);
int arm(Action& action, Telemetry& telemetry);
int land(Action& action, Telemetry& telemetry);
int take_off(Action& action);
int mission_mode(Mission& mission);
int mission_upload(Mission& mission);
int pause_mission(Mission& mission);
int rtl(Action& action);
int disarm(Action& action);
int set_rtl_altitude_m(Action& action, float alt);
int set_takeoff_altitude_m(Action& action, float alt);





void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

void component_discovered(ComponentType component_type)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Discovered a component with type "
              << unsigned(component_type) << std::endl;
}

int main(int argc, char** argv)
{
    Mavsdk mavsdk;
    std::string connection_url;
    ConnectionResult connection_result;

    bool discovered_system = false;
    if (argc == 2) {
        connection_url = argv[1];
        connection_result = mavsdk.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Connection failed: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    std::cout << "Waiting to discover system..." << std::endl;
    while (mavsdk.systems().size() == 0) {
     std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    mavsdk.subscribe_on_new_system([&mavsdk, &discovered_system]() {
        const auto system = mavsdk.systems().at(0);

        if (system->is_connected()) {
            std::cout << "Discovered system" << std::endl;
            discovered_system = true;
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2
    // seconds.
    sleep_for(seconds(2));

    if (!discovered_system) {
        std::cout << ERROR_CONSOLE_TEXT << "No system found, exiting." << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }

    const auto system = mavsdk.systems().at(0);

    // Register a callback so we get told when components (camera, gimbal) etc
    // are found.
    system->register_component_discovered_callback(component_discovered);

    auto action = Action{system};
    auto mission = Mission{system};
    auto telemetry = Telemetry{system};
    auto log_files = LogFiles(system);
    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready" << std::endl;
        sleep_for(seconds(1));
    }
    //SYNC----------------------------------------------------------------

    //We want to listen to the altitude of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = telemetry.set_rate_position(0.005);
    if (set_rate_result != Telemetry::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Setting rate failed:" << set_rate_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }
    //ASYNC-------------------------------------------------------------
    // std::cout << "Setting rate updates..." << '\n';

    // auto prom = std::make_shared<std::promise<Telemetry::Result>>();
    // auto future_result = prom->get_future();
    // // Set position update rate to 1 Hz.
    // telemetry.set_rate_position_async(1.0, [prom](Telemetry::Result result) {
    //     prom->set_value(result); //fulfill promise
    // });

    // //Block until promise is fulfilled (in callback function)
    // const Telemetry::Result result = future_result.get();
    // if (result != Telemetry::Result::Success) {
    //     // handle rate-setting failure (in this case print error)
    //     std::cout << "Setting telemetry rate failed (" << result << ")." << '\n';
    // }
    //-----------------------------------------------------------------

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry.subscribe_position([](Telemetry::Position position) {
    std::cout << "Altitude: " << position.relative_altitude_m << " m" << std::endl
              << "Latitude: " << position.latitude_deg << std::endl
              << "Longitude: " << position.longitude_deg << '\n';
    });

    // telemetry.subscribe_battery([](Telemetry::Battery battery){
    //     std::cout << "Battery Remaining Percentage: "<<battery.remaining_percent<<" "
    //               << "Battery: "<<battery.voltage_v<<std::endl;

    // });
    // telemetry.subscribe_gps_info([](Telemetry::GpsInfo gps_info){
    //     std::cout << "GPS INFO:FIX "<<gps_info.fix_type<<" && SAT "<<gps_info.num_satellites<<std::endl;
    // });
    // telemetry.subscribe_health([](Telemetry::Health health){
    //     std::cout << "HEALTH:ACC CAL OK "<<health.is_accelerometer_calibration_ok<<std::endl
    //               << "HEALTH:GLO POS OK"<<health.is_global_position_ok<<std::endl
    //               << "HEALTH:GYRO CAL OK "<<health.is_gyrometer_calibration_ok<<std::endl
    //               << "HEALTH:HOME POS OK "<<health.is_home_position_ok<<std::endl
    //               << "HEALTH:LEVEL CAL OK "<<health.is_level_calibration_ok<<std::endl
    //               << "HEALTH:LOCAL POS OK "<<health.is_local_position_ok<<std::endl
    //               << "HEALTH:MAG CAL OK "<<health.is_magnetometer_calibration_ok<<std::endl;
    // });
    // // telemetry.subscribe_odometry([](Telemetry::Odometry odometry){
    // //     std::cout << "ODOMETRY:ANG VEL "<< odometry.angular_velocity_body<<std::endl
    // //               << "ODOMETRY:CHILD FR ID "<< odometry.child_frame_id<<std::endl
    // //               << "ODOMETRY:FR ID "<< odometry.frame_id<<std::endl
    // //               << "ODOMETRY:POSE COVARIANCE "<< odometry.pose_covariance<<std::endl
    // //               << "ODOMETRY:POS BODY "<< odometry.position_body<<std::endl
    // //               << "ODOMETRY:Q "<< odometry.q<<std::endl
    // //               << "ODOMETRY:TIME uSEC "<< odometry.time_usec<<std::endl
    // //               << "ODOMETRY:VEL BODY "<< odometry.velocity_body<<std::endl
    // //               << "ODOMETRY:VEL COVARIANCE "<< odometry.velocity_covariance<<std::endl;
    // // });
    // // telemetry.subscribe_imu([](Telemetry::Imu imu){
    // //     std::cout << "IMU:ACC "<< imu.acceleration_frd<<std::endl
    // //               << "IMU:ANG VEL "<< imu.angular_velocity_frd<<std::endl
    // //               << "IMU:MAG "<< imu.magnetic_field_frd<<std::endl
    // //               << "IMU:TEMP "<< imu.temperature_degc<<std::endl;
    // // });
    // telemetry.subscribe_rc_status([](Telemetry::RcStatus rc_status){
    //     std::cout << "RCSTATUS: IS AVAILABLE "<<rc_status.is_available<<" SIGNAL "<<rc_status.signal_strength_percent<<std::endl;
    // });
    Telemetry::FlightMode oldFlightMode=Telemetry::FlightMode::Unknown;
telemetry.subscribe_flight_mode([&oldFlightMode](Telemetry::FlightMode flightMode) {
    if (oldFlightMode != flightMode) {
        //Flight mode changed. Print!
        std::cout << "FlightMode: " << flightMode << '\n';
        oldFlightMode=flightMode; //Save updated mode.
    }
    });


//     {
//     std::cout << "Waiting for system to be ready" << '\n';
//     auto prom = std::make_shared<std::promise<void>>();
//     auto future_result = prom->get_future();
//     telemetry.subscribe_health_all_ok(
//     [prom](bool result) {
//         //fulfill promise if health is OK
//         if (result) {// health OK
//             prom->set_value();
//         }
//     });

//     future_result.get(); //Block until promise is fulfilled.
//     // Now ready to arm
//     }

std::pair<Action::Result, float> t_alt = action.get_takeoff_altitude();
std::pair<Action::Result, float> rtl_alt = action.get_return_to_launch_altitude();
std::cout <<"Take off altitide: "<< t_alt.second <<"\n"<<"RTL altitude: "<< rtl_alt.second << std::endl;
sleep_for(seconds(3));
arm(action, telemetry);
take_off(action);
mission_upload(mission);
mission_mode(mission);
land(action, telemetry);
log_entries(log_files);
}
int arm(Action& action, Telemetry& telemetry)
{
    //Check if vehicle is ready to arm
    while (telemetry.health_all_ok() != true)
    {
        std::cout << "Vehicle is getting ready to arm" << std::endl;
        sleep_for(seconds(1));
    }
        // Arm vehicle
    std::cout << "Arming..." << std::endl;
    const Action::Result arm_result = action.arm();
        //handle_action_err_exit(arm_result, "Arm failed: ");

    if (arm_result != Action::Result::Success)
    {
        std::cout << ERROR_CONSOLE_TEXT << "Arming failed:" << arm_result << NORMAL_CONSOLE_TEXT
                  << std::endl;
    }
    return 1;
}
int rtl(Action& action)
{
    //RTL
    std::cout << "Changing to RTL..." << std::endl;
    const Action::Result rtl_result = action.return_to_launch();
    if (rtl_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "rtl failed:" << rtl_result<< NORMAL_CONSOLE_TEXT
                  << NORMAL_CONSOLE_TEXT << std::endl;
    }
    return 1;
}

int disarm(Action& action)
{
    //Disarm
    std::cout << "Disarming..." << std::endl;
    const Action::Result disarm_result = action.disarm();
    if (disarm_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Disarming failed:" << disarm_result<< NORMAL_CONSOLE_TEXT
                  << NORMAL_CONSOLE_TEXT << std::endl;
    }
    return 1;
}

int set_takeoff_altitude_m(Action& action, float alt)
{
    //Disarm
    std::cout << "Setting takeoff altitude to " <<alt<<" meters..."<<std::endl;
    const Action::Result set_alt_result = action.set_takeoff_altitude(alt);
    if (set_alt_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Setting Takeoff altitude failed:" << set_alt_result<< NORMAL_CONSOLE_TEXT
                  << NORMAL_CONSOLE_TEXT << std::endl;
    }
    return 1;
}
int set_rtl_altitude_m(Action& action, float alt)
{
    //Disarm
    std::cout << "Setting takeoff altitude to " <<alt<<" meters..."<<std::endl;
    const Action::Result set_rtl_result = action.set_return_to_launch_altitude(alt);
    if (set_rtl_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Setting RTL altitude failed:" << set_rtl_result<< NORMAL_CONSOLE_TEXT
                  << NORMAL_CONSOLE_TEXT << std::endl;
    }
    return 1;
}
int take_off(Action& action)
{
    //Take off
    std::cout << "Taking off..." << std::endl;
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Takeoff failed:" << takeoff_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
    }
    return 1;
}

int land(Action& action, Telemetry& telemetry)
{
    // Let it hover for a bit before landing again.
    sleep_for(seconds(5));

    std::cout << "Landing..." << std::endl;
    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Land failed:" << land_result << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }

    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing..." << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "Landed!" << std::endl;

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished..." << std::endl;
return 1;
}

int mission_mode(Mission& mission)
{
    // Before starting the mission, we want to be sure to subscribe to the mission progress.
    mission.subscribe_mission_progress([](Mission::MissionProgress mission_progress) {
        std::cout << "Mission status update: " << mission_progress.current << " / "
                  << mission_progress.total << std::endl;
    });
    {
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom->get_future();
        std::cout << "Starting mission." << std::endl;
        mission.start_mission_async([prom](Mission::Result result) {
            prom->set_value(result);
            std::cout << "Started mission." << std::endl;
        });
        const Mission::Result result = future_result.get();

    if (result != Mission::Result::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << "Mission start failed" << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
    while (!mission.is_mission_finished().second) {
        sleep_for(seconds(1));
    }
}
// {
//     auto prom = std::make_shared<std::promise<Mission::Result>>();
//     auto future_result = prom->get_future();
//     if (mission.is_mission_finished().second){
//         mission.clear_mission_async([prom](Mission::Result result){
//         std::cout << "Mission Finished & Cleared." << std::endl;
//             prom->set_value(result);
//     });
//     }
// }    
return 0;
    }

int pause_mission(Mission& mission)
{
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom->get_future();

        std::cout << "Pausing mission..." << std::endl;
        mission.pause_mission_async([prom](Mission::Result result) { prom->set_value(result); });

        const Mission::Result result = future_result.get();
        if (result != Mission::Result::Success) {
            std::cout << "Failed to pause mission (" << result << ")" << std::endl;
        } else {
            std::cout << "Mission paused." << std::endl;
        }
        return 1;
    }
int mission_upload(Mission& mission)
{       
    std::vector<Mission::MissionItem> mission_items;

    mission_items.push_back(make_mission_item(
        47.398170327054473,
        8.5456490218639658,
        10.0f,
        5.0f,
        false,
        20.0f,
        60.0f,
        Mission::MissionItem::CameraAction::None));

    mission_items.push_back(make_mission_item(
        47.398241338125118,
        8.5455360114574432,
        10.0f,
        2.0f,
        true,
        0.0f,
        -60.0f,
        Mission::MissionItem::CameraAction::TakePhoto));

    mission_items.push_back(make_mission_item(
        47.398139363821485,
        8.5453846156597137,
        10.0f,
        5.0f,
        true,
        -45.0f,
        0.0f,
        Mission::MissionItem::CameraAction::StartVideo));

    mission_items.push_back(make_mission_item(
        47.398058617228855,
        8.5454618036746979,
        10.0f,
        2.0f,
        false,
        -90.0f,
        30.0f,
        Mission::MissionItem::CameraAction::StopVideo));

    mission_items.push_back(make_mission_item(
        47.398100366082858,
        8.5456969141960144,
        10.0f,
        5.0f,
        false,
        -45.0f,
        -30.0f,
        Mission::MissionItem::CameraAction::StartPhotoInterval));

        std::cout << "Uploading mission..." << std::endl;
        // We only have the upload_mission function asynchronous for now, so we wrap it using
        // std::future.
        {
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom->get_future();
        Mission::MissionPlan mission_plan{};
        mission_plan.mission_items = mission_items;
        mission.upload_mission_async(
            mission_plan, [prom](Mission::Result result) { prom->set_value(result); });

        const Mission::Result result = future_result.get();
        if (result != Mission::Result::Success) {
            std::cout << "Mission upload failed (" << result << "), exiting." << std::endl;
            return 1;
        }
        std::cout << "Mission uploaded." << std::endl;    }
return 1;
}

Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    Mission::MissionItem::CameraAction camera_action)
{
    Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    new_item.is_fly_through = is_fly_through;
    new_item.gimbal_pitch_deg = gimbal_pitch_deg;
    new_item.gimbal_yaw_deg = gimbal_yaw_deg;
    new_item.camera_action = camera_action;
    return new_item;
}
int log_entries(LogFiles& log_files)
{
        auto prom = std::make_shared<std::promise<LogFiles::Result>>();
        auto future_result = prom->get_future();

        std::cout << "Getting Entries..." << std::endl;
        log_files.get_entries_async([prom](mavsdk::LogFiles::Result result, std::vector<mavsdk::LogFiles::Entry> vec){ 
            prom->set_value(result);
            std::cout<<"data="<<std::endl;

        for (auto& it : vec) { 
                std::cout <<"==>"<<it << ' '; 
        } 
            });

       const LogFiles::Result result = future_result.get();
        if (result == LogFiles::Result::Success) {
            std::cout << "getting entries-" << result << std::endl;
        } else {
            std::cout <<"getting entries failed" << std::endl;
        }
        return 1;
}