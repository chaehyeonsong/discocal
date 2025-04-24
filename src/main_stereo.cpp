#include <yaml-cpp/yaml.h>
#include "stereo.hpp"

int main(int argc, char** argv){
    string yaml_path="../config_example/stereo.yaml";
    if(argc ==2){
        yaml_path = string(argv[1]);
    }
    cout<<"yaml path: "<<yaml_path<<endl;

    YAML::Node node = YAML::LoadFile(yaml_path);
    StereoCalibration stereo_calibrator;
    stereo_calibrator.stereo_calibration(node);

    return 0;
}