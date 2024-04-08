#include "CTargetDetector.h"
double TargetDetector::ALPHA=0;
double TargetDetector::DELTA=0;



TargetDetector::TargetDetector(int n_x, int n_y, bool is_thermal, bool draw){
    this->n_x = n_x;    
    this->n_y = n_y;
    this->prev_success=false;
    this->is_thermal = is_thermal;
    
    this->color_threshold_min = 60; 
    this->size_threshold = 200;
    this->fullfill_threshold1 = 0.3; // rgb12: 0.1
    this->fullfill_threshold2 = 0.01;
    this->eccentricity_threshold = 0.1;

    this->draw=draw; 
    this->use_weight=false; 
    this->do_iterative_search=true;
    if(is_thermal){
        this->color_threshold_max = 150; 
        this->color_threshold_step = 5;
    }
    else{
        if(do_iterative_search){
            this->color_threshold_max = 255; 
            this->color_threshold_step = 5;
        }
        else this->color_threshold_max = 125;
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
        if(do_iterative_search){
            color_threshold = color_threshold_max;
            while(color_threshold>color_threshold_min){
                ret=detect_circles(img, target);
                if(ret) break;
                else color_threshold -= color_threshold_step;
            }
            color_threshold = 125;
            ret=detect_circles(img, target,true);
        }
        else{
            color_threshold = color_threshold_max;
            ret=detect_circles(img, target, true);
        }
        if(this->draw){
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
        printf("color_threshold: %d\n",color_threshold);
        if(ret) prev_success=true;
    }
    else{
        throw WrongTypeException();
    }


    return make_pair(ret, target);
}

void TargetDetector::dfs(cv::Mat img, vector<vector<bool>> &buffer, vector<array<int,3>> &area, int x, int y){
    int W = img.cols;
    int H = img.rows;
    if(area.size()>W*H/4) return;
    stack<cv::Point2i> stack;
    buffer[y][x]=false;
    stack.push(cv::Point2i(x,y));
    while(stack.size()!=0){
        cv::Point2i curr_pt = stack.top();
        stack.pop();
        int x{curr_pt.x}, y{curr_pt.y};
        if( check_pixel(img, x, y)){
            area.push_back(array<int,3>{x,y,img.data[y*W+x]});
            if(x+1<W && buffer[y][x+1]) { buffer[y][x+1]=false; stack.push(cv::Point2i(x+1,y));}
            if(x>0 && buffer[y][x-1]) {buffer[y][x-1]=false; stack.push(cv::Point2i(x-1,y));}
            if(y+1<H && buffer[y+1][x]) {buffer[y+1][x]=false; stack.push(cv::Point2i(x,y+1));}
            if(y>0 && buffer[y-1][x]) {buffer[y-1][x]=false; stack.push(cv::Point2i(x,y-1));}
        }

    }
}
bool TargetDetector::check_pixel(cv::Mat img, int x,int y){
    int W = img.cols;
    return img.data[y*W+x] < color_threshold;
}

bool TargetDetector::ellipse_test(const vector<array<int,3>> &area, cv::Point2f &pt){
    //Todo fullfill test
    
    int n = area.size();

    if(n<size_threshold) return false;

    int x,y, maxx{-1},maxy{-1},minx{10000},miny{10000};

    double mx{0},my{0},cx{0}, cy{0},xx{0},xy{0},yy{0},weight_sum{0};
    double weight;
    for(int i=0;i<n;i++){
        if(use_weight) weight = (color_threshold-area[i][2]);
        else weight=1.0;

        x = area[i][0];
        y = area[i][1];
        // int weight = 1;
        minx = x<minx ? x: minx;
        miny = y<miny ? y: miny;
        maxx = x>maxx ? x: maxx;
        maxy = y>maxy ? y: maxy;
        cx+= x*weight;
        cy+= y*weight;
        mx+=x;
        my+=y;
        xx += x*x*1;
        yy += y*y*1;
        xy += x*y*1;
        
        weight_sum += weight;
    }

    double ratio1 = abs(1- (maxx-minx)*(maxy-miny)/4*M_PI/n);

    if( ratio1>fullfill_threshold1) {
        // if(draw) printf("fial ratio1: %f\n",ratio1);
        return false;
    }
    cx = cx/weight_sum;
    cy = cy/weight_sum;
    mx = mx/n;
    my = my/n;
    xx = xx/n - pow(mx,2);
    xy = xy/n - mx*my;
    yy = yy/n - pow(my,2);

    double det = (xx+yy)*(xx+yy)-4*(xx*yy-xy*xy);
    if (det > 0) det = sqrt(det); else det = 0;
    double f0 = ((xx+yy)+det)/2;
    double f1 = ((xx+yy)-det)/2;
    double m0 = sqrt(f0);
    double m1 = sqrt(f1);

    double ratio2 = abs(1- m0*m1*4*M_PI/n);

    if(ratio2>fullfill_threshold2) {
        // if(draw) printf("fial ratio2: %f\n",ratio2);
        return false;
    }

    if(m1/m0<eccentricity_threshold) {
        // if(draw) printf("fial ratio3: %f\n",m1/m0);
        return false;
    }

    pt.x = cx;
    pt.y = cy;

    return true;
}

bool TargetDetector::compare(cv::Point2i a, cv::Point2i b){
    double c  = cos(ALPHA);
    double s  = sin(ALPHA);
    double x1mx2 = c*(a.x-b.x)+s*(a.y-b.y);
    double y1my2 = -s*(a.x-b.x)+c*(a.y-b.y);
    bool aless; 

    if(abs(y1my2)<DELTA){
        aless = x1mx2<0;
    }
    else{
        aless = y1my2<0;
    }
	return aless; // true이면 a가 b 앞에 감. 즉 a가 b보다 앞일 조건 쓰면 됌
}

bool TargetDetector::detect_circles(cv::Mat img, vector<cv::Point2f>&target, bool debug){
    int W = img.cols;
    int H = img.rows;

    cv::Mat bgr_img;
    if(draw) cv::cvtColor(img,bgr_img, cv::COLOR_GRAY2BGR);
    vector<cv::Point2f> source;


    // initialize
    vector<vector<bool>> buffer; // 확인 안했으면 true;
    buffer.reserve(H);
    for(int i=0;i<H;i++){
        vector<bool> temp(W,true);
        buffer.push_back(temp);
    }
    vector<array<int,3>> area;
    cv::Point2f pt;
    for(int y=0;y<H;y++){
        for(int x=0;x<W;x++){
            if(buffer[y][x] && check_pixel(img,x,y)){

                area.clear();
                dfs(img,buffer,area,x,y);
                if(ellipse_test(area,pt)){
                    if(draw){
                        for(int i=0;i<area.size();i++){
                            int pos= area[i][0]+area[i][1]*W;
                            if (use_weight) bgr_img.data[3*pos]=(color_threshold-area[i][2]);
                            else bgr_img.data[3*pos] = 100;
                            bgr_img.data[3*pos+1]=0;
                            bgr_img.data[3*pos+2]=0;
                        }
                    }
                    source.push_back(pt);
                }
            }
        }
    }

    // find axis
    if (debug && draw){
        if(is_thermal) cv::resize(bgr_img, bgr_img, cv::Size(bgr_img.cols*2, bgr_img.rows*2));
        else cv::resize(bgr_img, bgr_img, cv::Size(bgr_img.cols*0.8, bgr_img.rows*0.8));
        cv::imshow("input_image",bgr_img);
        return  false;
    }
    else{
        if(source.size()==n_x*n_y) {
            sortTarget(source,target);
            if(target.size()==n_x*n_y) {
                if(draw){
                    
                    for(int i=0;i<target.size();i++){
                        
                        cv::Point2f pt = target[i];
                        if(i/4 ==0 ) cv::putText(bgr_img,to_string(i%4),pt,0,1,cv::Scalar(0,255,0),2);
                        else if(i/4==1) cv::putText(bgr_img,to_string(i%4),pt,0,1,cv::Scalar(0,0,255),2);
                        else if(i/4==2) cv::putText(bgr_img,to_string(i%4),pt,0,1,cv::Scalar(0,255,255),2);
                        // cv::imwrite("../temp.png",bgr_img);
                        // usleep(200000);
                    }
                    if(is_thermal) cv::resize(bgr_img, bgr_img, cv::Size(bgr_img.cols*2, bgr_img.rows*2));
                    else cv::resize(bgr_img, bgr_img, cv::Size(bgr_img.cols*0.8, bgr_img.rows*0.8));
                    cv::imshow("input_image",bgr_img);
                }
                return  true;
            }
            else return false;
        }
        else return false;
    }

}


void TargetDetector::sortTarget(vector<cv::Point2f>&source, vector<cv::Point2f>&dist){
    CircleGridFinder gridfinder(false);
    gridfinder.findGrid(source, cv::Size(n_x,n_y),dist);
    reverse(dist.begin(), dist.end());
    return;
}

