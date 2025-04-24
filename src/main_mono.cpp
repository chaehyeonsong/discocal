#include <yaml-cpp/yaml.h>
#include "mono.hpp"

int main(int argc, char** argv){
    string yaml_path="../config_example/mono.yaml";
    if(argc ==2){
        yaml_path = string(argv[1]);
    }
    cout<<"yaml path: "<<yaml_path<<endl;

    YAML::Node node = YAML::LoadFile(yaml_path);
    MonoCalibration mono_calibrator;
    mono_calibrator.mono_calibration(node);
    return 0;
}