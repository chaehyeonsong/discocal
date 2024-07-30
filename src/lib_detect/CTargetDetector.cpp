#include "CTargetDetector.h"

TargetDetector::TargetDetector(int n_x, int n_y, bool draw){
    this->n_x = n_x;    
    this->n_y = n_y;
    
    this->size_threshold = 400;

    //for conic test
    this->fullfill_threshold = 0.01;
    this->eccentricity_threshold = 0.1;

    this->draw=draw; 
    this->drawing_scale= 0.5;

    for(int i=0; i<(n_y+6)/7;i++){
        text_colors.push_back(cv::Scalar(255,0,0));
        text_colors.push_back(cv::Scalar(0,255,0));
        text_colors.push_back(cv::Scalar(0,0,255));
        text_colors.push_back(cv::Scalar(255,255,0));
        text_colors.push_back(cv::Scalar(255,0,255));
        text_colors.push_back(cv::Scalar(0,255,255));
        text_colors.push_back(cv::Scalar(255,255,255));
    }
}

cv::Mat TargetDetector::preprocessing(cv::Mat img){
    cv::Mat filltered_img, gray_img;
    cv::bilateralFilter(img,filltered_img,-1,10,10);
    if(filltered_img.channels()==3){
        cv::cvtColor(filltered_img, gray_img, cv::COLOR_BGR2GRAY);
        return gray_img;
    }
    else if(filltered_img.channels()==1){
        return filltered_img;
    }
    else{
        throw WrongTypeException();
    }
    
}

pair<bool,vector<cv::Point2f>> TargetDetector::detect(cv::Mat img, string type){
    bool ret;
    vector<cv::Point2f> target;
    if (type == "square"){
        // 순서 : y 높은순 ->  x 높은순
        ret = cv::findChessboardCorners(img,cv::Size(n_x,n_y), target); //flag????
        if(draw&&ret){
            cv::Mat bgr_img;
            cv::cvtColor(img,bgr_img, cv::COLOR_GRAY2BGR);
            cv::drawChessboardCorners(bgr_img, cv::Size(n_x,n_y),target,ret);
            cv::imshow("hi",bgr_img);
            cv::waitKey(0);
        }
        reverse(target.begin(), target.end());
    }
    else if(type == "circle"){

        if(this->draw){
            ret= detect_circles(img, target, true);
            printf("save: 1, ignore: 0\n");
            char key = cv::waitKey(0);
            while(key != '0' && key!='1'){
                printf("wrong commend is detected\n");
                key = cv::waitKey(0);
            }
            cv::destroyAllWindows();
            if(key == '0'){
                ret = false;
            }
        }
        else{
            ret= detect_circles(img, target);
        }
    }
    else{
        throw WrongTypeException();
    }

    return make_pair(ret, target);
}

bool TargetDetector::ellipse_test(const cv::Moments &moments){
    double mx{0},my{0},xx{0},xy{0},yy{0},area{0};
    area = moments.m00;

    if(area < size_threshold) return false;

    mx= moments.m10 / area;
    my= moments.m01 / area;
    xx = moments.mu20 / area;
    xy = moments.mu11 / area;
    yy = moments.mu02 / area;
    
    double det = (xx+yy)*(xx+yy)-4*(xx*yy-xy*xy);
    if (det > 0) det = sqrt(det); else det = 0;
    double f0 = ((xx+yy)+det)/2;
    double f1 = ((xx+yy)-det)/2;
    double m0 = sqrt(f0);
    double m1 = sqrt(f1);

    double ratio2 = abs(1- m0*m1*4*M_PI/area);

    if(ratio2>fullfill_threshold) {
        // if(draw) printf("fial ratio2: %f\n",ratio2);
        return false;
    }

    if(m1/m0<eccentricity_threshold) {
        // if(draw) printf("fial ratio3: %f\n",m1/m0);
        return false;
    }

    return true;
}

void TargetDetector::sortTarget(vector<cv::Point2f>&source, vector<cv::Point2f>&dist){
    CircleGridFinder gridfinder(false);
    gridfinder.findGrid(source, cv::Size(n_x,n_y),dist);
    reverse(dist.begin(), dist.end());
    return;
}

std::vector<int> TargetDetector::get_randomSample(int range, int n){
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
bool TargetDetector::circle_compare( circle_info circle1,  circle_info circle2){
    return circle1.first.m00>circle2.first.m00;
}

bool TargetDetector::detect_circles(cv::Mat img, vector<cv::Point2f>&target, bool debug){

    cv::Mat img_origin, img_blur, img_thresh, img_morph, img_output;

    img_origin = img.clone();
    img_output = img_origin.clone();
    cvtColor(img_output, img_output, cv::COLOR_GRAY2BGR);

    // collect all circular bolb candidates //
    int blur_size = min(img_origin.rows, img_origin.cols)/500*2+1;
    vector<circle_info> circle_candidates;
    cv::GaussianBlur(img_origin, img_blur, cv::Size(blur_size, blur_size), 0);
    for (int bs = 3; bs <= 19; bs += 2)
    {
        cv::adaptiveThreshold(img_blur, img_thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, bs, 2);
        for (int s = 1; s <= 5; s += 2)
        {
            
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(s, s));
            cv::morphologyEx(img_thresh, img_morph, cv::MORPH_CLOSE, kernel);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(img_morph, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (size_t i = 0; i < contours.size(); i++)
            {
                cv::Moments moments = cv::moments(contours[i]);
                if(ellipse_test(moments)){
                    circle_candidates.push_back(circle_info(moments, contours[i]));
                }
            }
        }
    }
    
    // merge same blobs //
    vector<circle_info> circles;
    for (size_t i = 0; i < circle_candidates.size(); i++)
    {
        cv::Moments m1 = circle_candidates[i].first;
        cv::Point m1_c = cv::Point(int(m1.m10 / m1.m00), int(m1.m01 / m1.m00));

        bool isnew=true;

        for (size_t j = 0; j < circles.size(); j++)
        {
            cv::Moments m2 = circles[j].first;

            cv::Point m2_c = cv::Point(int(m2.m10 / m2.m00), int(m2.m01 / m2.m00));

            float dist = sqrt(pow(m1_c.x - m2_c.x, 2) + pow(m1_c.y - m2_c.y, 2));
            if (dist < 5.)
            {
                isnew = false;

                if(m1.m00 >= m2.m00){
                    circles[j].first = m1;
                    circles[j].second = circle_candidates[i].second;
                }
                break;
            }  

        }
            
        if(isnew)  {
            circles.push_back(circle_candidates[i]);
        }
    }

    // select largest n_x * n_y blobs // 
    sort(circles.begin(), circles.end(), circle_compare);
    std::vector<cv::Point2f> source;
    std::vector<std::vector<cv::Point>> circle_contours;
    for(int i =0;i< n_x*n_y;i++){
        cv::Moments m1 = circles[i].first;
        cv::Point2f pt;
        pt.x = (float) m1.m10/m1.m00, 
        pt.y = (float) m1.m01/m1.m00;
        source.push_back(pt);

        circle_contours.push_back(circles[i].second);
    }

    
    // sorting and filltering blobs to a grid structure //
    if(debug) {
        cv::drawContours(img_output, circle_contours, -1, cv::Scalar(100, 0, 0), cv::FILLED);
        cv::Mat img_output2;
        cvtColor(img_origin, img_output2, cv::COLOR_GRAY2BGR);
        img_output = img_output*0.5 + img_output2*0.5;
    }

    sortTarget(source,target);
    bool result=false;
    if(target.size()==n_x*n_y){
        result=true;
        if(debug){
            for(int i=0;i<target.size();i++)
            {
                cv::Point2f pt = target[i];
                for (int j=0;j<n_x;j++)
                {
                    if(i/n_x == j ) cv::putText(img_output,to_string(i%n_x),pt,0,1,text_colors[j],2);
                }
            }
        }
    }
    if(debug)
    {
        cv::resize(img_output, img_output, cv::Size(img_output.cols*drawing_scale, img_output.rows*drawing_scale));
        float bottom = img_output.rows*0.95;
        float left = img_output.cols*0.05;
        if(result) cv::putText(img_output,"save: 1, ignore: 0",cv::Point2f(left,bottom),0,2,cv::Scalar(0,0,255),5);
        else cv::putText(img_output,"detection fail, press any key",cv::Point2f(left,bottom),0,2,cv::Scalar(0,0,255),5);
        cv::imshow("input_image",img_output);
    }
    
    return result;

}
