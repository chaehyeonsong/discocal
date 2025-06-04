#include "CTargetDetector.h"
TargetDetector::TargetDetector(int n_x, int n_y, bool draw){
    this->n_x = n_x;    
    this->n_y = n_y;
    this->numerical_stable = 1e-8;
    
    this->size_threshold = 100; //default 100

    //for conic test
    this->fullfill_threshold = 0.01; //default 0.01
    this->eccentricity_threshold = 0.9; //default 0.9
    this->distance_threshold = 10; // minimum pixels between two circles

    this->draw=draw; 
    this->drawing_scale= 1;

    for(int i=0; i<(n_y+4)/5;i++){
        // text_colors.push_back(cv::Scalar(255,0,0));
        text_colors.push_back(cv::Scalar(0,255,0));
        text_colors.push_back(cv::Scalar(0,0,255));
        text_colors.push_back(cv::Scalar(255,255,0));
        text_colors.push_back(cv::Scalar(255,0,255));
        text_colors.push_back(cv::Scalar(0,255,255));
        // text_colors.push_back(cv::Scalar(255,255,255));
    }
}

cv::Mat TargetDetector::preprocessing(const cv::Mat img, string detection_mode){
    cv::Mat output_img;
    if(img.channels()==3){
        
        if(detection_mode == "saturation"){
            cv::Mat hsv_img;
            cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
            cv::extractChannel(hsv_img, output_img, 1);
        }
        else{
            cv::cvtColor(img, output_img, cv::COLOR_BGR2GRAY);
        }
        
        return output_img;
    }
    else if(img.channels()==1){
        cv::bilateralFilter(img,output_img,-1,10,10);
        return output_img;
    }
    else{
        throw WrongTypeException();
    }
    
}
void TargetDetector::update_autocorrelation(cv::Mat &src, vector<Shape>& control_shapes){
    cv::Mat grad_x3, grad_y3;
    cv::Scharr(src, grad_x3,CV_32F,1,0);
    cv::Scharr(src, grad_y3,CV_32F,0,1);
    grad_x3 = grad_x3/16; 
    grad_y3 = grad_y3/16;

    cv::Mat _Wxx, _Wxy, _Wyy;

    _Wxx = grad_x3.mul(grad_x3);
    _Wxy = grad_x3.mul(grad_y3);
    _Wyy = grad_y3.mul(grad_y3);

    cv::Mat Wxx, Wxy, Wyy;

    int blur_size=3;
    cv::GaussianBlur(_Wxx, Wxx, cv::Size(blur_size, blur_size), 0);
    cv::GaussianBlur(_Wxy, Wxy, cv::Size(blur_size, blur_size), 0);
    cv::GaussianBlur(_Wyy, Wyy, cv::Size(blur_size, blur_size), 0);

    for(int i=0; i< control_shapes.size();i++){
        int y_c = int(control_shapes[i].y);
        int x_c = int(control_shapes[i].x);
        double wxx = Wxx.at<float>(y_c, x_c);
        double wxy = Wxy.at<float>(y_c, x_c);
        double wyy = Wyy.at<float>(y_c, x_c);
        double det = wxx*wyy-wxy*wxy;

        // int window_size=5;
        // int delta = intensity_range(src, x_c,y_c,window_size);

        control_shapes[i].Kxx = wyy/det;
        control_shapes[i].Kxy = -wxy/det;
        control_shapes[i].Kyy = wxx/det;
    }
}

pair<bool,vector<Shape>> TargetDetector::detect(cv::Mat& img, string type){
    if(this->draw){
        this->drawing_scale = 1000.0/max(img.rows, img.cols);
    }
    bool ret;
    vector<Shape> control_shapes;
    pair<bool,vector<Shape>> final_result;
    cv::Mat bgr_img, output_img;
    cv::cvtColor(img,bgr_img, cv::COLOR_GRAY2BGR);
    if (type == "square"){
        // 순서 : y 높은순 ->  x 높은순
        vector<cv::Point2f> corners;
        ret = cv::findChessboardCorners(img,cv::Size(n_x,n_y), corners); //flag????
        if(draw&&ret){
            output_img = bgr_img;
            cv::drawChessboardCorners(output_img, cv::Size(n_x,n_y),corners,ret);
        }
        reverse(corners.begin(), corners.end());
        for(int i=0; i<corners.size();i++) control_shapes.push_back(Shape(corners[i].x,corners[i].y,0));
        update_autocorrelation(img, control_shapes);
    }
    else if(type == "circle"){
        int output_col= img.cols*this->drawing_scale;
        int output_row = img.rows*this->drawing_scale;
        cv::resize(bgr_img, output_img, cv::Size(output_col, output_row));
        
        ret= detect_circles(img, output_img,control_shapes, draw);
    }
    else{
        throw WrongTypeException();
    }
    final_result.first = ret;
    final_result.second = control_shapes;

    visualize_result(output_img, final_result);
    return final_result;
}

void TargetDetector::save_result(string path){
    cv::imwrite(path, this->detection_result);
}

void TargetDetector::visualize_result(cv::Mat& img_output, pair<bool,vector<Shape>> &result){
    // visualize
    int scale= 100/this->drawing_scale;
    if(result.first){
        for(int i=0;i< result.second.size();i++)
        {
            Shape shape = result.second[i];
            cv::Point2f pt(shape.x*this->drawing_scale,shape.y*this->drawing_scale);
            for (int j=0;j<n_x;j++)
            {
                if(i/n_x == j ) {
                    string message = to_string(i);
                    cv::putText(img_output,message,pt,0,1,text_colors[j],2);
                }
            }
            //draw covariance
            if(shape.uncertainty()!=0){
                array<double,3> ellipse = cov2ellipse(shape.Kxx, shape.Kxy, shape.Kyy);
                double a{ellipse[0]}, b{ellipse[1]}, angle{ellipse[2]};
                // a = a*10;
                // b = b*10;
                cv::RotatedRect rRect = cv::RotatedRect(pt, cv::Size2f(a*scale,b*scale), angle);
                cv::ellipse(img_output, rRect,  cv::Scalar(0,255,255));
                cv::String message = cv::format("%.2f, %.2f", round(a*100)/100.0, round(b*100)/100.0);
                cv::putText(img_output,message,cv::Point2f(pt.x-20,pt.y+30),0,0.5,cv::Scalar(0,0,255));
            }
        }
    }

    int output_col= img_output.cols;
    int output_row = img_output.rows;
    float bottom = img_output.rows*0.95;
    float left = img_output.cols*0.05;
    double fontscale = 1.2;
    int thickness = 3;
    if(this->draw){
        if(result.first) cv::putText(img_output,"save: 1, ignore: 0, quit_visualize: q",cv::Point2f(left,bottom),0,fontscale,cv::Scalar(0,0,255),thickness);
        else cv::putText(img_output,"detection fail, press any key",cv::Point2f(left,bottom),0,fontscale,cv::Scalar(0,0,255),thickness);
        this->detection_result = img_output;

        cv::imshow("input_image",img_output);
        printf("Interact with the visualization window\n");
        char key = cv::waitKey(0);
        
        if(key != '0' && key!='1' && key!='q'){
            printf("wrong commend is detected\n");
            key = cv::waitKey(0);
        }
        cv::destroyAllWindows();
        if(key == '0'){
            result.first = false;
        }
        if(key == 'q'){
            this->draw = false;
        }
    }
    else{
        if(!result.first) cv::putText(img_output,"detection fail",cv::Point2f(left,bottom),0,fontscale,cv::Scalar(0,0,255),thickness);
        this->detection_result = img_output;
    }

}


bool TargetDetector::ellipse_test(const Shape &shape){
    double area = shape.m00;

    if(area < size_threshold) return false;

    double mx= shape.m10/area;
    double my= shape.m01/area;
    double xx = shape.m20/area - mx*mx;
    double xy = shape.m11/area - mx*my;
    double yy = shape.m02/area - my*my;
    
    double det = (xx+yy)*(xx+yy)-4*(xx*yy-xy*xy);
    if (det > 0) det = sqrt(det); else det = 0;
    double f0 = ((xx+yy)+det)/2;
    double f1 = ((xx+yy)-det)/2;
    double m0 = sqrt(f0);
    double m1 = sqrt(f1);

    double ratio1 = abs(1-m1/m0);
    double ratio2 = abs(1- m0*m1*4*M_PI/area);
    // printf("ratio1: %f, ratio2: %f\n",ratio1, ratio2);

    if(ratio2>fullfill_threshold) {
        // if(draw) printf("fial ratio2: %f\n",ratio2);
        return false;
    }

    if(ratio1>eccentricity_threshold) {
        // if(draw) printf("fial ratio3: %f\n",m1/m0);
        return false;
    }

    return true;
}

void TargetDetector::sortTarget(vector<cv::Point2f>&source, vector<cv::Point2f>&dist){
    bool isAsymmetricGrid=false;
    CircleGridFinder gridfinder(isAsymmetricGrid);
    gridfinder.findGrid(source, cv::Size(n_x,n_y),dist);
    reverse(dist.begin(), dist.end());
    return;
}

bool TargetDetector::circle_compare( circle_info circle1,  circle_info circle2){
    return circle1.first.m00>circle2.first.m00;
}

int TargetDetector::intensity_range(const  cv::Mat &gray_img, int x, int y, float step_x, float step_y, int step_length){
    int height = gray_img.rows;
    int width = gray_img.cols;
    int min = 255;
    int max = 0;

    for(int i = -step_length; i<=step_length; i++){
        int xi = x + round(step_x*i);
        int yi = y + round(step_y*i);
        xi = xi>0 ? xi : 0;
        xi = xi<width ? xi : width-1;
        yi = yi>0 ? yi : 0;
        yi = yi<height ? yi : height-1;
        int value = gray_img.at<uchar>(yi, xi);
        max = value > max ? value : max;
        min = value < min ? value : min;        
    }
    
    return max-min;
}

int TargetDetector::intensity_range(const  cv::Mat &gray_img, int x, int y, int window_size){
    int height = gray_img.rows;
    int width = gray_img.cols;
    int min = 255;
    int max = 0;

    for(int i = -window_size; i<=window_size; i++){
        for(int j = -window_size; j<=window_size; j++){
            int xi = x + i;
            int yi = y + j;
            xi = xi>0 ? xi : 0;
            xi = xi<width ? xi : width-1;
            yi = yi>0 ? yi : 0;
            yi = yi<height ? yi : height-1;
            int value = gray_img.at<uchar>(yi, xi);
            max = value > max ? value : max;
            min = value < min ? value : min;
        }
    }
    
    return max-min;
}


Shape TargetDetector::contour2shape(const vector<cv::Point2i> &contour){
    int n = contour.size();
    // int x_c = contour[n-1].x;
    // int y_c = contour[n-1].y;

    int sign = 1;
    double m00{0}, m10{0}, m01{0}, m20{0},m11{0},m02{0};
    double M00{0}, M10{0}, M01{0}, M20{0},M11{0},M02{0};

    for(int i=0; i<n;i++){
        cv::Point2i pt = contour[i];
        int x_f = pt.x;
        int y_f = pt.y;

        int x_c = contour[n-1].x;;
        int y_c = contour[n-1].y;;
        if(i!=0){
            pt = contour[i-1];
            x_c = pt.x;
            y_c = pt.y;
        }
        double dxy= x_f*y_c - x_c * y_f;
        int Mx = x_f+x_c;
        int My = y_f+y_c;

        M00 += dxy;
        M10 += dxy*Mx;
        M01 += dxy*My;
        M20 += dxy*(x_c*Mx+x_f*x_f);
        M11 += dxy*(x_c*(My+y_c)+x_f*(My+y_f));
        M02 += dxy*(y_c*My+y_f*y_f);

        // x_c = x_f;
        // y_c = y_f;
    }

    if(M00<0) sign=-1;

    m00 = sign*M00/2;
    m10 = sign*M10/6;
    m01 = sign*M01/6;
    m20 = sign*M20/12;
    m11 = sign*M11/24;
    m02 = sign*M02/12;

    Shape shape{n,m00,m10,m01,m20,m11,m02};

    return shape;
}

bool TargetDetector::cal_shape_cov(const vector<cv::Point2i> &contour, Shape* shape,  const  cv::Mat &gray_img, const  cv::Mat &grad_x3, const cv::Mat &grad_y3){
    int step_length=10;
    int width = gray_img.cols;
    int n = contour.size();
    Eigen::ArrayXd J00(2*n);
    Eigen::ArrayXd J10(2*n);
    Eigen::ArrayXd J01(2*n);
    Eigen::ArrayXXd Jc(2,2*n);
    J00.setZero();
    J10.setZero();
    J01.setZero();
    Jc.setZero();
    
    double s = pow(1,2);
    double z=1/s*2/3;
    float * grad_x3_ptr = (float*) grad_x3.data;
    float * grad_y3_ptr = (float*) grad_y3.data;

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(10 * n);
    #pragma omp parallel
    {
        std::vector<Eigen::Triplet<double>> local_triplets;

        #pragma omp for nowait
        for (int i = 0; i < n; i++) {
            int curr_x_index = 2 * i;
            int curr_y_index = (curr_x_index + 1) % (2 * n);
            int next_x_index = (curr_x_index + 2) % (2 * n);
            int next_y_index = (curr_x_index + 3) % (2 * n);

            local_triplets.emplace_back(curr_x_index, next_x_index, -z);
            local_triplets.emplace_back(next_x_index, curr_x_index, -z);
            local_triplets.emplace_back(curr_y_index, next_y_index, -z);
            local_triplets.emplace_back(next_y_index, curr_y_index, -z);
        }

        #pragma omp for nowait
        for (int i = 0; i < n; i++) {
            int i_p = (i - 1 + n) % n;
            int i_c = i;
            int i_f = (i + 1) % n;

            cv::Point2i pt_p = contour[i_p];
            cv::Point2i pt_c = contour[i_c];
            cv::Point2i pt_f = contour[i_f];

            int x_p = pt_p.x, y_p = pt_p.y;
            int x_c = pt_c.x, y_c = pt_c.y;
            int x_f = pt_f.x, y_f = pt_f.y;

            J00(2 * i_c) = y_p - y_f;
            J00(2 * i_c + 1) = x_f - x_p;
            J10(2 * i_c) = x_f * (y_c - y_f) + 2 * x_c * (y_p - y_f) + x_p * (y_p - y_c);
            J10(2 * i_c + 1) = (x_f - x_p) * (x_f + x_c + x_p);
            J01(2 * i_c) = (y_p - y_f) * (y_f + y_c + y_p);
            J01(2 * i_c + 1) = y_f * (x_f - x_c) + 2 * y_c * (x_f - x_p) + y_p * (x_c - x_p);

            int pos = y_c * width + x_c;
            float gx_i = grad_x3_ptr[pos];
            float gy_i = grad_y3_ptr[pos];
            float gn_i = sqrt(gx_i * gx_i + gy_i * gy_i) + numerical_stable;
            float step_x = gx_i / gn_i;
            float step_y = gy_i / gn_i;
            double delta = intensity_range(gray_img, x_c, y_c, step_x, step_y, step_length) + 0.0;

            Eigen::Matrix2d V, W, likelihood_i;
            V << step_x, -step_y,
                step_y,  step_x;
            W << pow(gn_i * 4 / delta, 2), 0,
                0, 0;
            likelihood_i = V * W * V.transpose();

            int idx = 2 * i_c;
            local_triplets.emplace_back(idx,     idx,     likelihood_i(0,0) + 2 * z + numerical_stable);
            local_triplets.emplace_back(idx + 1, idx,     likelihood_i(1,0));
            local_triplets.emplace_back(idx,     idx + 1, likelihood_i(0,1));
            local_triplets.emplace_back(idx + 1, idx + 1, likelihood_i(1,1) + 2 * z + numerical_stable);
        }

        #pragma omp critical
        triplets.insert(triplets.end(), local_triplets.begin(), local_triplets.end());
    }
    Eigen::SparseMatrix<double> sparse_p_info(2 * n, 2 * n);
    sparse_p_info.setFromTriplets(triplets.begin(), triplets.end());


    double M00 = shape->m00*2;
    Jc.row(0) = (J10/3-J00*shape->x)/(M00+numerical_stable);
    Jc.row(1) = (J01/3-J00*shape->y)/(M00+numerical_stable);

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> chol(sparse_p_info);
    Eigen::Matrix2d c_cov = Jc.matrix() *chol.solve(Jc.matrix().transpose());


    if (isnan(c_cov(0,0)) || isnan(c_cov(1,0)) || isnan(c_cov(1,1))) return false;
    double scale_factor=1.0; // default: 1.0
    shape->Kxx = c_cov(0,0)*scale_factor;
    shape->Kxy = c_cov(0,1)*scale_factor;
    shape->Kyy = c_cov(1,1)*scale_factor;
    return true;
}

array<double,3> TargetDetector::cov2ellipse(double Kxx, double Kxy, double Kyy){
    Eigen::Matrix2d cov;
    cov << Kxx, Kxy, Kxy, Kyy;
    Eigen::BDCSVD<Eigen::MatrixXd> svd(cov,Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::VectorXd singular_values=svd.singularValues();

    double m0 = max(0.0, singular_values(0));
    double m1 = max(0.0, singular_values(1));
    double a = 2*sqrt(m0);
    double b = 2*sqrt(m1);

    double angle = atan2(V(1,0),V(0,0))*180/M_PI;
    
    return array<double, 3> {a, b, angle};
}

bool TargetDetector::detect_circles(cv::Mat& img, cv::Mat& img_output,vector<Shape>&target, bool debug){

    cv::Mat img_origin, img_blur;
    // cv::Mat grad_x3, grad_y3,grad_x5, grad_y5;
    img_origin = img.clone();    

    // collect all circular bolb candidates //


    cv::Mat edge_img, grad_x3, grad_y3;
    cv::Scharr(img_origin, grad_x3,CV_32F,1,0);
    cv::Scharr(img_origin, grad_y3,CV_32F,0,1);
    grad_x3 = grad_x3/32; // check
    grad_y3 = grad_y3/32;

    std::vector<std::vector<cv::Point>> contours;
    vector<circle_info> circle_candidates;
    int contour_min_length=10;

    
    cv::Mat img_thresh, img_morph;
    int scale_factor = max(img_origin.rows, img_origin.cols)/1000+1;
    int blur_size = scale_factor*2+3; //should be odd
    // cv::GaussianBlur(img_origin, img_blur, cv::Size(blur_size, blur_size), -1);
    img_blur = img_origin;

    // threshold based
    vector<int> block_sizes = {6*scale_factor+1,12*scale_factor+1,18*scale_factor+1};
    vector<int> kernel_sizes = {2*scale_factor-1,4*scale_factor-1};
    for (int bs : block_sizes)
    {
        cv::adaptiveThreshold(img_blur, img_thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, bs, 2);
        // cv::threshold(img_blur, thr, 255, 255, cv::THRESH_BINARY_INV);
        for (int s :kernel_sizes)
        {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(s, s));
            cv::morphologyEx(img_thresh, img_morph, cv::MORPH_CLOSE, kernel);    
            std::vector<std::vector<cv::Point>> all_contours;
            cv::findContours(img_morph, all_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            int count=0;
            
            for(std::vector<cv::Point> contour : all_contours)
            {
                if(contour.size()<contour_min_length) continue;
                Shape shape= contour2shape(contour);
                
                if(ellipse_test(shape)){      
                    if(cal_shape_cov(contour, &shape, img_origin, grad_x3, grad_y3)){
                        shape.ks=s;
                        shape.bs=bs;    
                        circle_candidates.push_back(circle_info(shape, contour));
                        count++;
                    }
                    
                }
            }
            if(count==0){
                cv::findContours(img_morph, all_contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
                for(std::vector<cv::Point> contour : all_contours)
                {
                    if(contour.size()<contour_min_length) continue;
                    Shape shape= contour2shape(contour);
                    
                    if(ellipse_test(shape)){      
                        if(cal_shape_cov(contour, &shape, img_origin, grad_x3, grad_y3)){
                            shape.ks=s;
                            shape.bs=bs;    
                            circle_candidates.push_back(circle_info(shape, contour));
                        }
                        
                    }
                }
            }
        }
    }


    // struct timeval  tv;
    // double begin, end;
    // gettimeofday(&tv, NULL);
    // begin = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
    // gettimeofday(&tv, NULL);
    // end = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
    // double duration =(end - begin) / 1000;
    // printf("finishied. Runtime: %.2f\n", duration);
    // finish = clock();
    // double duration = (double)(finish - start) / CLOCKS_PER_SEC;
    // printf("%f초\n", duration);


    // merge same blobs and select largest n_x * n_y blobs//
    sort(circle_candidates.begin(), circle_candidates.end(), circle_compare);
    vector<circle_info> circles;
    for (size_t i = 0; i < circle_candidates.size(); i++)
    {
        Shape s1 = circle_candidates[i].first;

        bool isnew=true;

        for (size_t j = 0; j < circles.size(); j++)
        {
            Shape s2 = circles[j].first;

            float dist = sqrt(pow(s1.x - s2.x, 2) + pow(s1.y - s2.y, 2));
            if (dist < distance_threshold)
            {
                isnew = false;

                if(s1.uncertainty() < s2.uncertainty()){
                // if(s1.m00 > s2.m00){
                    circles[j].first = s1;
                    circles[j].second = circle_candidates[i].second;
                }
                break;
            }  

        }
            
        if(isnew && circles.size()<n_x*n_y)  {
            circles.push_back(circle_candidates[i]);
        }
    }

    if(debug) {
        std::vector<std::vector<cv::Point>> circle_contours;
        for(circle_info circle : circles){
            std::vector<cv::Point> contour = circle.second;
            for(int i=0;i< contour.size();i++){
                contour[i].x*= this->drawing_scale;
                contour[i].y*= this->drawing_scale;
            }
            circle_contours.push_back(contour);
        }
        // cv::drawContours(img_output, circle_contours, -1, cv::Scalar(100, 0, 0), cv::FILLED);
        // cv::Mat img_output2;
        // cv::cvtColor(img_origin, img_output2, cv::COLOR_GRAY2BGR);
        // img_output = img_output*0.5 + img_output2*0.5;

        cv::drawContours(img_output, circle_contours, -1, cv::Scalar(255, 0, 0), cv::LINE_4);
    }


    // sort circles
    std::vector<cv::Point2f> source;
    vector<cv::Point2f>dest;
    for(circle_info circle : circles){
        Shape s1 = circle.first;
        cv::Point2f pt(s1.x,s1.y);
        source.push_back(pt);
    }
    sortTarget(source,dest);
    for(int i=0; i<dest.size();i++){
        double u(dest[i].x), v(dest[i].y);
        for(int j=0; j<circles.size();j++){
            Shape s_t = circles[j].first;
            double dist = sqrt(pow(s_t.x - u, 2) + pow(s_t.y - v, 2));
            if (dist < distance_threshold){
                target.push_back(s_t);
                break;
            }
        }
    }


    // visualize
    bool result=false;
    if(target.size()==n_x*n_y) result=true;    
    return result;

}

cv::Mat TargetDetector::translation_blur(const cv::Mat &img, double trans){
    if(trans<1) return img.clone();

    int step=10;
    double d_trans= 2*trans/(step-1.0);
    int width = img.cols;
    int height = img.rows;

    cv::Mat motion_blurred_img(height, width, CV_64F, cv::Scalar(0.0));
    bool is_x_direction = rand()%2 ==0 ? true : false;
    // bool is_x_direction=true;

    for(int i =0; i<step;i++){
        double dx = d_trans*i;
        cv::Mat M ;
        M = cv::Mat_<double>({ 2, 3 }, { 1, 0, dx, 0, 1, 0 });
        // if(is_x_direction) M = cv::Mat_<double>({ 2, 3 }, { 1, 0, dx, 0, 1, 0 });
        // else M = cv::Mat_<double>({ 2, 3 }, { 1, 0, 0, 0, 1, dx });
        cv::Mat translated_img;
        cv::warpAffine(img,translated_img,M,cv::Size(width,height),cv::INTER_CUBIC,cv::BORDER_CONSTANT,cv::Scalar(255.0));
        translated_img.convertTo(translated_img, CV_64F,1.0,0.0);
        motion_blurred_img =motion_blurred_img+ translated_img;
        
    }
    motion_blurred_img = motion_blurred_img /(step+0.0);

    motion_blurred_img.convertTo(motion_blurred_img, CV_8U, 1.0, 0.0);
    return motion_blurred_img;
}

cv::Mat TargetDetector::rotation_blur(const cv::Mat &img, double dtheta){
    if(dtheta<1) return img.clone();

    int step=10;
    double d_rot= 2*dtheta/(step-1.0);

    int width = img.cols;
    int height = img.rows;

    cv::Mat motion_blurred_img(height, width, CV_64F, cv::Scalar(0.0));
    // cv::Point2f center(0, 0);
    cv::Point2f center(width / 2.0, height / 2.0);

    for(int i =0; i<step;i++){
        cv::Mat M = cv::getRotationMatrix2D(center, -dtheta+d_rot*i, 1.0);

        cv::Mat translated_img;
        cv::warpAffine(img,translated_img,M,cv::Size(width,height),cv::INTER_CUBIC,cv::BORDER_CONSTANT,cv::Scalar(255.0));
        translated_img.convertTo(translated_img, CV_64F,1.0,0.0);
        motion_blurred_img =motion_blurred_img+ translated_img;
    }

    motion_blurred_img = motion_blurred_img /(step+0.0);
    motion_blurred_img.convertTo(motion_blurred_img, CV_8U, 1.0, 0.0);

    return motion_blurred_img;
}

