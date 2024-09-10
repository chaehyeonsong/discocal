#ifndef __UTILS_H__
#define __UTILS_H__

#include <yaml-cpp/yaml.h>

struct Shape{
    double x, y;
    double Kxx, Kxy, Kyy;
    double area;
    Shape(double _x, double _y, double _Kxx=0, double _Kxy=0, double _Kyy=0, double _area=0) : x(_x), y(_y),Kxx(_Kxx), Kxy(_Kxy), Kyy(_Kyy), area(_area){};
};

struct Params{
    double fx;
    double fy;
    double cx;
    double cy;
    double skew;
    double d[4];
    double radius;

    Params(double _fx=0, double _fy=0, double _cx=0, double _cy=0, double _skew=0, double _d1=0, double _d2=0, double _d3=0, double _d4=0, double _radius=0){
        fx = _fx;
        fy = _fy;
        cx = _cx;
        cy = _cy;
        skew = _skew;
        d[0] = _d1;
        d[1] = _d2;
        d[2] = _d3;
        d[3] = _d4;
        radius = _radius;
    }

    Params(YAML::Node camera_node){
        fx = camera_node["fx"].as<double>();
        fy = camera_node["fy"].as<double>();
        cx = camera_node["cx"].as<double>();
        cy = camera_node["cy"].as<double>();
        skew = camera_node["skew"].as<double>();
        d[0] = camera_node["d1"].as<double>();
        d[1] = camera_node["d2"].as<double>();
        d[2] = camera_node["d3"].as<double>();
        d[3] = camera_node["d4"].as<double>();
    }
};

struct Point{
    double x;
    double y;
    Point(double _x, double _y) : x(_x), y(_y){};
};

static bool endsWith(std::string const &str, std::string const &suffix) {
    if (str.length() < suffix.length()) {
        return false;
    }
    return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

static std::vector<int> random_sampling(int range, int n){
    std::vector<bool> data(range);
    fill(data.begin(),data.end(), true);

    std::vector<int> sampledData;
    srand((unsigned int)time(NULL));
    while(n>0){
        int num =rand()%range;
        if(data[num]){
            sampledData.push_back(num);
            data[num]=false;
            n--;
        }
    }
    return sampledData;
}
 
template <typename T>
static std::vector<T> split(std::string str, char Delimiter) {
    std::istringstream iss(str);             // istringstream에 str을 담는다.
    std::string buffer;                      // 구분자를 기준으로 절삭된 문자열이 담겨지는 버퍼
 
    std::vector<T> result;
 
    // istringstream은 istream을 상속받으므로 getline을 사용할 수 있다.
    while (getline(iss, buffer, Delimiter)) {
        std::stringstream value(buffer);
        T d;
        value>>d;
        result.push_back(d);               // 절삭된 문자열을 vector에 저장
    }
 
    return result;
}

#endif