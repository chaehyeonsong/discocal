#ifndef __UTILS_H__
#define __UTILS_H__

#include <yaml-cpp/yaml.h>
#include <tabulate/table.hpp>
#include "ceres/ceres.h"
#include <filesystem>
#include <iostream>


static void print_process(int count, int MAX, std::string prefix = ""){
    const char bar = '='; // 프로그레스바 문자  
	const char blank = ' '; // 비어있는 프로그레스바 문자  
	// const int MAX = 20; // 프로그레스바 길이  
	int i; // 반복문 전용 변수  
	int bar_count; // 프로그레스바 갯수 저장 변수  
	float percent; // 퍼센트 저장 변수  
    printf("\r%s%d/%d [", prefix.c_str(),count, MAX); // 진행 상태 출력  
    percent = (float)count/MAX*100; // 퍼센트 계산  
    bar_count = count; // 프로그레스바 갯수 계산  
    for(i=0; i<MAX; i++) { // LEN길이의 프로그레스바 출력  
        if(bar_count > i) { // 프로그레스바 길이보다 i가 작으면 
            printf("%c", bar);
        } else { // i가 더 커지면  
            printf("%c", blank);
        }
    }
    printf("] %0.2f%%", percent); // 퍼센트 출력  
    // std::cout<<std::flush;
    std::cout<<std::endl;
}

static void mkdir( const std::string &path ) {
    std::filesystem::path p( path );

    if ( std::filesystem::is_directory( p ) )
        return;

    std::filesystem::create_directories( p );
}

class OutofRangeError: public std::exception{
    public:
        const char* what(){
            return "Value exceed the range";
        }
};
class YamlfileError: public std::exception{
    public:
        const char* what(){
            return "Wrong yaml field";
        }
};
class LackOfImageError: public std::exception{
    public:
        const char* what(){
            return "Total number of images is too small. Correct more images";
        }
};
template <typename T>
static std::vector<T> split(std::string str, char Delimiter) {
    std::istringstream iss(str);
    std::string buffer;

    std::vector<T> result;

    while (std::getline(iss, buffer, Delimiter)) {
        std::stringstream value(buffer);
        T d;
        value >> d;
        result.push_back(d);
    }

    return result;
}

static std::string to_stringf(double value, int pos)
{   
    double r_value = round(value*pow(10, pos))/ (pow(10,pos)+0.0);
    std::string num = std::to_string(r_value);
	return num.substr(0, num.find('.') + pos + 1);
}

struct Shape{
    double x, y;
    double m00, m10, m01, m20, m11, m02;
    double Kxx, Kxy, Kyy;
    int_least32_t n;
    int ks;
    int bs;
    Shape(double _cx, double _cy, double _area){
        x = _cx;
        y = _cy;
        m00 = _area;
        m10 = m00*x;
        m01 = m00*y;
        m20= _area/(4*M_PI);
        m11=0;
        m02=_area/(4*M_PI);
        Kxx=0;
        Kxy=0;
        Kyy=0;
        n=0;
    }
    Shape(int _n, double _m00, double _m10, double _m01, double _m20, double _m11, double _m02, double _Kxx=0, double _Kxy=0, double _Kyy=0){
        n  = _n;
        m00= _m00;
        m10= _m10;
        m01= _m01;
        m20= _m20;
        m11= _m11;
        m02= _m02;
        Kxx = _Kxx;
        Kxy = _Kxy;
        Kyy = _Kyy;

        x = m10/m00;
        y = m01/m00;
    }
    double uncertainty(){
        double det = Kxx*Kyy-Kxy*Kxy;
        return 0.5*(log(2*M_PI*det)+1.0);
        // return Kxx+Kxy;
        // return Kxx*Kyy-Kxy*Kxy;
    }
    std::string to_string(){
        std::string str = std::to_string(x) + "\t"+std::to_string(y) + "\t"+std::to_string(Kxx) + "\t"+std::to_string(Kxy) + "\t"+std::to_string(Kyy);
        return str;
    }
};


template<typename T>
struct Params_T {
    T fx;
    T fy;
    T cx;
    T cy;
    T skew;
    std::array<T, 4> d;
    
    T s_fx;
    T s_fy;
    T s_cx;
    T s_cy;
    T s_skew;
    std::array<T, 4> s_d;

    // Default constructor
    Params_T(T _fx = T(0), T _fy = T(0), T _cx = T(0), T _cy = T(0), T _skew = T(0),
           T _d1 = T(0), T _d2 = T(0), T _d3 = T(0), T _d4 = T(0))
    : fx(_fx), fy(_fy), cx(_cx), cy(_cy), skew(_skew) {
        d[0] = _d1;
        d[1] = _d2;
        d[2] = _d3;
        d[3] = _d4;
        // initialize_unc();
    }
    
    // YAML constructor
    Params_T(YAML::Node camera_node) {
        fx = static_cast<T>(camera_node["fx"].as<double>());
        fy = static_cast<T>(camera_node["fy"].as<double>());
        cx = static_cast<T>(camera_node["cx"].as<double>());
        cy = static_cast<T>(camera_node["cy"].as<double>());
        skew = static_cast<T>(camera_node["skew"].as<double>());
        d[0] = static_cast<T>(camera_node["d1"].as<double>());
        d[1] = static_cast<T>(camera_node["d2"].as<double>());
        d[2] = static_cast<T>(camera_node["d3"].as<double>());
        d[3] = static_cast<T>(camera_node["d4"].as<double>());
        // initialize_unc();
    }

    void update_unc(const std::array<double, 9>& params_cov) {
        s_fx = static_cast<T>(params_cov[0]);
        s_fy = static_cast<T>(params_cov[1]);
        s_cx = static_cast<T>(params_cov[2]);
        s_cy = static_cast<T>(params_cov[3]);
        s_skew = static_cast<T>(params_cov[4]);
        s_d[0] = static_cast<T>(params_cov[5]);
        s_d[1] = static_cast<T>(params_cov[6]);
        s_d[2] = static_cast<T>(params_cov[7]);
        s_d[3] = static_cast<T>(params_cov[8]);
    }
    
    void initialize_unc() {
        std::array<T, 9> params_cov;
        params_cov.fill(T(0));
        update_unc(params_cov);
    }
    
    int get_precision(T s) {
        int max_precision = 0;
        int min_precision = -4;
        for (int i = max_precision; i >= min_precision; i--) {
            T mod = ceres::pow(T(10), T(i));
            T v = ceres::floor(s / mod);
            if (v != T(0)) {
                return -i;
            }
        }
        return 4;
    }

    std::string sigma3(T x, T s_x) {
        if (std::is_same_v<T, double>) { // only double type
            int n = get_precision(s_x);
            int toralance = 3;
            return to_stringf(x, n) + "\u00B1" + to_stringf(T(toralance) * s_x, n + 1);
        }
        else return std::string();
    }
    
    std::string to_string() {
        std::string str = "fx\tfy\tcx\tcy\tskew\td1\td2\td3\td4\n";
        str += std::to_string(static_cast<double>(fx)) + "\t";
        str += std::to_string(static_cast<double>(fy)) + "\t";
        str += std::to_string(static_cast<double>(cx)) + "\t";
        str += std::to_string(static_cast<double>(cy)) + "\t";
        str += std::to_string(static_cast<double>(skew)) + "\t";
        str += std::to_string(static_cast<double>(d[0])) + "\t";
        str += std::to_string(static_cast<double>(d[1])) + "\t";
        str += std::to_string(static_cast<double>(d[2])) + "\t";
        str += std::to_string(static_cast<double>(d[3]));
        return str;
    }

    tabulate::Table to_table(bool only_intrinsic = false, bool details = false) {
        tabulate::Table table;
        if (std::is_same_v<T, double>) { // only double type
            int toralance = 2;
            if (only_intrinsic) {
                table.add_row({"Parameter", "fx", "fy", "cx", "cy", "skew"});
                table.add_row({"mean", to_stringf(fx, 2), to_stringf(fy, 2), to_stringf(cx, 2), to_stringf(cy, 2), to_stringf(skew, 2)});
            } else {
                table.add_row({"Parameter", "fx", "fy", "cx", "cy", "skew", "d1", "d2", "d3", "d4"});
                table.add_row({"mean", to_stringf(fx, 2), to_stringf(fy, 2), to_stringf(cx, 2), to_stringf(cy, 2), to_stringf(skew, 2),
                               to_stringf(d[0], 3), to_stringf(d[1], 3), to_stringf(d[2], 3), to_stringf(d[3], 4)});
                if (details) {
                    table.add_row({"2sigma", to_stringf(T(toralance) * s_fx, 3), to_stringf(T(toralance) * s_fy, 3),
                                   to_stringf(T(toralance) * s_cx, 3), to_stringf(T(toralance) * s_cy, 3), to_stringf(T(toralance) * s_skew, 3),
                                   to_stringf(T(toralance) * s_d[0], 4), to_stringf(T(toralance) * s_d[1], 4),
                                   to_stringf(T(toralance) * s_d[2], 4), to_stringf(T(toralance) * s_d[3], 5)});
                }
            }
        }
        return table;
    }
};

typedef Params_T<double> Params;

// struct Point{
//     double x;
//     double y;
//     Point(double _x, double _y) : x(_x), y(_y){};
// };

template<typename T>
struct Point_T {
    T x, y;
    Point_T(T _x, T _y) : x(_x), y(_y) {};
};

typedef Point_T<double> Point;

static bool endsWith(std::string const &str, std::string const &suffix) {
    if (str.length() < suffix.length()) {
        return false;
    }
    return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

static std::vector<int> sorted_random_sampling(int range, int n, unsigned int seed=(unsigned int)time(NULL) ){
    if(range<n){
        throw OutofRangeError();
    }
    std::vector<bool> data(range);
    fill(data.begin(),data.end(), true);

    std::vector<int> sampledData;
    srand(seed);
    while(n>0){
        int num =rand()%range;
        if(data[num]){
            sampledData.push_back(num);
            data[num]=false;
            n--;
        }
    }
    sort(sampledData.begin(), sampledData.end());
    return sampledData;
}

static double cdf(double x)
{
    // constants
    double a1 =  0.254829592;
    double a2 = -0.284496736;
    double a3 =  1.421413741;
    double a4 = -1.453152027;
    double a5 =  1.061405429;
    double p  =  0.3275911;

    // Save the sign of x
    int sign = 1;
    if (x < 0)
        sign = -1;
    x = fabs(x)/sqrt(2.0);

    // A&S formula 7.1.26
    double t = 1.0/(1.0 + p*x);
    double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

    return 0.5*(1.0 + sign*y);
}

static bool check_img_path(std::string path){
    bool check=false;
    check = path.find(".png") != std::string::npos || path.find(".PNG") != std::string::npos \
    || path.find(".jpeg") != std::string::npos || path.find(".JPEG") != std::string::npos \
    || path.find(".jpg") != std::string::npos || path.find(".JPG") != std::string::npos ;
    return check;
}

#endif