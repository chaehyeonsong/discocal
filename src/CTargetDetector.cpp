#include "CTargetDetector.h"

TargetDetector::TargetDetector(int n_x, int n_y, bool draw){
    this->n_x = n_x;    
    this->n_y = n_y;
    
    this->size_threshold = 400;

    //for conic test
    this->fullfill_threshold = 0.01;
    this->eccentricity_threshold = 0.1;

    this->draw=draw; 
    this->drawing_scale= 1.0;

    this->text_colors;
    text_colors.push_back(cv::Scalar(255,0,0));
    text_colors.push_back(cv::Scalar(0,255,0));
    text_colors.push_back(cv::Scalar(0,0,255));
    text_colors.push_back(cv::Scalar(255,255,0));
    text_colors.push_back(cv::Scalar(255,0,255));
    text_colors.push_back(cv::Scalar(0,255,255));
    text_colors.push_back(cv::Scalar(255,255,255));
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

bool TargetDetector::detect_circles(cv::Mat img, vector<cv::Point2f>&target, bool debug){

    cv::Mat img_origin, img_blur, img_thresh, img_morph, img_contour, img_output;

    img_origin = img.clone();
    img_output = img_origin.clone();
    cvtColor(img_output, img_output, cv::COLOR_GRAY2BGR);

    // collect all circular bolb candidates //
    std::vector<std::vector<cv::Point>> circle_contour_candidates;
    std::vector<cv::Moments> circle_moments_candidates;
    cv::GaussianBlur(img_origin, img_blur, cv::Size(5, 5), 0);
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
                    circle_moments_candidates.push_back(moments);
                    circle_contour_candidates.push_back(contours[i]);
                }
            }
        }
    }
    
    // merge same blobs //
    std::vector<cv::Moments> circle_moments;
    std::vector<std::vector<cv::Point>> circle_contours;
    for (size_t i = 0; i < circle_moments_candidates.size(); i++)
    {
        cv::Moments m1 = circle_moments_candidates[i];

        bool isnew=true;

        for (size_t j = 0; j < circle_moments.size(); j++)
        {
            cv::Moments m2 = circle_moments[j];

            cv::Point m1_c = cv::Point(int(m1.m10 / m1.m00), int(m1.m01 / m1.m00));
            cv::Point m2_c = cv::Point(int(m2.m10 / m2.m00), int(m2.m01 / m2.m00));

            float dist = sqrt(pow(m1_c.x - m2_c.x, 2) + pow(m1_c.y - m2_c.y, 2));
            if (dist < 5.)
            {
                isnew = false;

                if(m1.m00 >= m2.m00){
                    circle_moments[j] = m1;
                    circle_contours[j] = circle_contour_candidates[i];
                }
                break;
            }  

        }
            
        if(isnew)  {
            circle_moments.push_back(m1);
            circle_contours.push_back(circle_contour_candidates[i]);
        }
    }

    // sorting and filltering blobs to a grid structure //
    std::vector<cv::Point2f> source;
    cv::Point2f pt;
    for(size_t i =0; i<circle_moments.size();i++){
        cv::Moments m1 = circle_moments[i];
        pt.x = (float) m1.m10/m1.m00, 
        pt.y = (float) m1.m01/m1.m00;
        source.push_back(pt);
    }
    
    if(debug) cv::drawContours(img_output, circle_contours, -1, cv::Scalar(100, 0, 0), cv::FILLED);

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
        cv::imshow("input_image",img_output);
    }
    
    return result;

}
