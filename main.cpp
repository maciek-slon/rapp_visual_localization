#include <rapp/cloud/service_controller/service_controller.hpp>
#include <rapp/cloud/navigation/visual_localization/visual_localization.hpp>

#include <rapp-robots-api/vision/vision.hpp>
#include <rapp-robots-api/communication/communication.hpp>
#include <rapp-robots-api/navigation/navigation.hpp>

//#define DEBUG

namespace rr = rapp::robot;

void status_cb(int status) {
    if (status != 0)
        std::cout << "Error!\n";
}

void callback(std::vector<std::string> names, std::vector<rapp::object::point> centers, std::vector<float> scores, int result) {
    std::cout << "Found " << names.size() << " obejcts\n";
}

int main(int argc, char * argv[]) {
    rr::communication com(argc, argv);
    rr::vision vis(argc, argv);
    rr::navigation nav(argc, argv);
    
    // look straight ahead
    nav.take_predefined_posture("Stand", 0.3);
    nav.move_joint({"HeadYaw","HeadPitch"}, {0.0f, 0.0f}, 0.5);

    // take picture of empty scene
    auto pict = vis.capture_image(0, rapp::robot::vision::vga4, "png");

//  rapp::cloud::platform_info info = {"155.207.19.229", "9001", "rapp_token"}; 
    rapp::cloud::platform_info info = {"192.168.18.186", "9001", "rapp_token"}; 
    rapp::cloud::service_controller ctrl(info);

    int id = -1;

    ctrl.make_call<rapp::cloud::visual_localization_init>("map1.xml", [&](int ret_id){id = ret_id;});
    
    std::cout << "Working with map " << id << "\n";
    
    rapp::object::point pose_delta(0, 0, 0);
    
    ctrl.make_call<rapp::cloud::visual_localization>(id, pict, pose_delta, [&](rapp::object::point pt, float bel, int st){
        std::cout << "Pose: " << pt.x << "," << pt.y << "," << pt.z << " (bel: " << bel << ")\n";
    });
    
    ctrl.make_call<rapp::cloud::visual_localization>(id, pict, pose_delta, [&](rapp::object::point pt, float bel, int st){
        std::cout << "Pose: " << pt.x << "," << pt.y << "," << pt.z << " (bel: " << bel << ")\n";
    });
    
    ctrl.make_call<rapp::cloud::visual_localization>(id, pict, pose_delta, [&](rapp::object::point pt, float bel, int st){
        std::cout << "Pose: " << pt.x << "," << pt.y << "," << pt.z << " (bel: " << bel << ")\n";
    });

    return 0;
}
