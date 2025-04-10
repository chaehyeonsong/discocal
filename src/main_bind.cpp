#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "mono.hpp"
#include "stereo.hpp"

namespace py = pybind11;

void mono_calibration_py(const std::string& config_path) {
    YAML::Node node = YAML::LoadFile(config_path);
    MonoCalibration mono_calibrator;
    mono_calibrator.mono_calibration(node);
}

void stereo_calibration_py(const std::string& config_path) {
    YAML::Node node = YAML::LoadFile(config_path);
    StereoCalibration stereo_calibrator;
    stereo_calibrator.stereo_calibration(node);
}

PYBIND11_MODULE(pydiscocal, m) {
    m.doc() = "Python binding for Discocal";
    m.def("mono_calibration", &mono_calibration_py, "A function to calibrate mono camera");
    m.def("stereo_calibration", &stereo_calibration_py, "A function to calibrate stereo camera");
}