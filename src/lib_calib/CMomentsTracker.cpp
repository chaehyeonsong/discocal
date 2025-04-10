#include "CMomentsTracker.h"

MomentsTracker::MomentsTracker(int dim)
{
    this-> max_dim = dim;
    this->max_n = max_dim*3;
    this->comb_max = max_n*2+1;
    init();
}

MomentsTracker::MomentsTracker(){
    this-> max_dim = 4;
    this->max_n = max_dim*3;
    this->comb_max = max_n*2+1;
    init();
}
MomentsTracker::~MomentsTracker()
{
}

void MomentsTracker::init(){
    comb_buffer.reserve(comb_max+1);
    for(int i=0; i<comb_max+1;i++){
        vector<int> temp(i+1,0);
        comb_buffer.push_back(temp);
        comb_buffer[i][0]=1;
        comb_buffer[i][i]=1;
    }

    for(int i=1;i<comb_max+1;i++){
        for(int j=1;j<i;j++){
            comb_buffer[i][j]=comb_buffer[i-1][j-1]+comb_buffer[i-1][j];
        }
    }

    // init moment //
    moment_buffer.reserve(max_n+1);
    for(int i=0; i<max_n+1;i++){
        vector<double> temp(max_n-i+1,0);
        moment_buffer.push_back(temp);
        for(int j=0; j< max_n-i+1;j++){
            moment_buffer[i][j]= _M_2i2j(i,j);
        }
    }

}

double MomentsTracker::nCr(int n, int r){
    if(n> comb_max || n<r) throw MomentsTrackerError();
    return (double)(comb_buffer[n][r]);
}
double MomentsTracker::_M_2i2j(int i, int j){
    // a, b == 1
    double den= nCr(2*(i+j),i+j)*nCr(i+j,i);
    double num= nCr(2*(i+j),2*i)*(i+j+1)*pow(2,2*(i+j));
    // cout<<den<<num<<endl;
    return den/num;
    
}
double MomentsTracker::M_2i2j(int i, int j){
    if(i> max_n || j>max_n) throw MomentsTrackerError();
    else return moment_buffer[i][j];
    
}
double MomentsTracker::M_2i2j(int i, int j, double a, double b){
    // return M^{2i2j}
    return M_2i2j(i,j)*pow(a,2*i)*pow(b,2*j);
}


array<double ,3> MomentsTracker::intSn(int n,double a, double b, double tx, double ty, double alpha){
    // is validated by wolframAlpha
    double tx_s = cos(alpha)*tx+sin(alpha)*ty;
    double ty_s = -sin(alpha)*tx+cos(alpha)*ty;
    double s0=0,sx=0,sy=0;
    array<double ,3> results;

    for(int i=0;i<n+1;i++){
        for(int j=0;j<n-i+1;j++){
            double M = M_2i2j(i,j,a,b);
            double m_0 =0;
            double m_x =0;
            double m_y =0;
            for(int k=i;k<n-j+1;k++){
                m_0+= nCr(n,k)*nCr(2*k,2*i)*nCr(2*(n-k),2*j)*pow(tx_s,2*(k-i))*pow(ty_s,2*(n-j-k));
                m_x+= nCr(n,k)*nCr(2*k+1,2*i)*nCr(2*(n-k),2*j)*pow(tx_s,2*(k-i)+1)*pow(ty_s,2*(n-j-k));
                m_y+= nCr(n,k)*nCr(2*k,2*i)*nCr(2*(n-k)+1,2*j)*pow(tx_s,2*(k-i))*pow(ty_s,2*(n-j-k)+1);
            
            }
            
            s0 += m_0*M;
            sx += m_x*M;
            sy += m_y*M;
            // printf("%d, %d : %e, %e, %e, %e\n",i,j,M,m_0,m_x,m_y);
        }
    }

    // support plane to normal plane
    results[0] = s0;
    results[1] = cos(alpha)*sx - sin(alpha)*sy;
    results[2] = sin(alpha)*sx + cos(alpha)*sy;
    return results;

}
double MomentsTracker::C0n(int n,vector<double> ds){
    double result=0;
    int n_d =max_dim;
    for(int i=max(0,n-n_d);i<min(n,n_d)+1;i++){
        result+= (2*i+1)*ds[i]*ds[n-i];
    }
    return result;
}
double MomentsTracker::C1n(int n,vector<double> ds){
    double result=0;
    int n_d =max_dim;
    for(int i=max(0,n-n_d*2);i<min(n,n_d)+1;i++){
        double temp =0;
        for(int j=max(0,n-i-n_d); j<min(n-i,n_d)+1;j++){
            temp+= ds[j]*ds[n-i-j];
        }
        result+= (2*i+1)*ds[i]*temp;
    }
    return result;
}

array<double, 5> MomentsTracker::ellipse2array(Eigen::Matrix3d Q){
    double a=Q(0,0), b=Q(0,1), c=Q(1,1), d=Q(0,2), e=Q(1,2), f=Q(2,2);
    double x,y,m0,m1,v0,v1;
    double det_Q = Q.determinant();
    double det_Q33 =  a*c-b*b;
    double s = a+c;
    x = (-c*d+b*e)/det_Q33;
    y = (b*d-a*e)/det_Q33;

    double k = - det_Q/det_Q33;
    double root = sqrt(pow(a-c,2)+4*pow(b,2));
    double lamda0 = (s-root)/2;
    double lamda1 = (s+root)/2;
    m0 = sqrt(k/lamda0);
    m1 = sqrt(k/lamda1);
    if (b==0){
        v0 = 0;
        v1 = 1;
    }
    else{
        v0 = ((c-a)+root)/2;
        v1 = -b;
    }
    double alpha = atan2(v1,v0);

    array<double, 5> result ={x,y,m0,m1,alpha};
    return result;
}

Point MomentsTracker::ne2dp(Eigen::Matrix3d Q, vector<double> ds){
    /*
    input: ellips in normal plane and distortion parameter
    output: centor point of region of distorted ellipse
    */

    array<double, 5> ellipse = ellipse2array(Q);

    double a,b,tx,ty,alpha;
    tx = ellipse[0];
    ty = ellipse[1];
    a = ellipse[2];
    b = ellipse[3];
    alpha = ellipse[4];
    double A_d=0; // =Ad/An
    double x_d=0;
    double y_d=0;

    for(int n=0;n<3*max_dim+1;n++){
        double c_0n = C0n(n,ds);
        double c_1n = C1n(n,ds);
        array<double,3> all_Sn = intSn(n,a,b,tx,ty,alpha);
        A_d+= c_0n * all_Sn[0];
        x_d += c_1n * all_Sn[1];
        y_d += c_1n * all_Sn[2];
    }

    x_d = x_d / A_d;
    y_d = y_d / A_d;
    return Point(x_d, y_d);
}

Point MomentsTracker::distort_Point(Point pn, vector<double> ds){
    double x, s, y, k;
    x = pn.x;
    y = pn.y;
    s = x*x+y*y;
    k = 0;
    for( int i=0;i<max_dim+1;i++){
        k += ds[i]*pow(s,i);
    }
    return Point(k*x, k*y);
}

Point MomentsTracker::wc2dp_Numerical(Eigen::Matrix3d Cw, Eigen::Matrix3d H, vector<double> ds, int total_iter){
    /*
    input: ellips in normal plane and distortion parameter
    output: centor point of region of distorted ellipse
    */

    // int total_iter =pow(5,2);
    int iter = (int)(sqrt(total_iter));
    double A_d=0; // =Ad/An
    double x_d=0;
    double y_d=0;

    double t_x = -Cw(0,2);
    double t_y = -Cw(1,2);
    double R = sqrt(-Cw(2,2)+pow(t_x,2)+pow(t_y,2));
    double r_step = 1.0/iter;
    double t_step = 2*M_PI/iter; // t == theta



    for(int i=0;i<iter+1;i++){
        for(int j=0;j<iter+1;j++){
            // int scale_i=2;
            // int scale_j=2;
            // if(i==0 || i==iter) scale_i=1;
            // if(j==0 || j==iter) scale_j=1;
            // double r = r_step*(i);
            // double t = t_step*(j);
            double r = r_step*(i+0.5);
            double t = t_step*(j+0.5);

            double x_w = t_x + R*r*cos(t);
            double y_w = t_y + R*r*sin(t);
            double f = H(0,0)*x_w + H(0,1)*y_w + H(0,2);
            double g = H(1,0)*x_w + H(1,1)*y_w + H(1,2);
            double h = H(2,0)*x_w + H(2,1)*y_w + H(2,2);
            double x_n = f/h;
            double y_n = g/h;

            double H02 = H(1,0)*H(2,1)-H(1,1)*H(2,0);
            double H12 = H(0,0)*H(2,1)-H(0,1)*H(2,0);
            double H22 = H(0,0)*H(1,1)-H(0,1)*H(1,0);

            double Jwrt = R*R*r;

            double Jnw = (H02*f/h -H12*g/h+H22)/pow(h,2);

            // double x_n = tx+a*r*cos(alpha)*cos(t)-b*r*sin(alpha)*sin(t);
            // double y_n = ty+a*r*sin(alpha)*cos(t)+b*r*cos(alpha)*sin(t);
            double s_n = pow(x_n,2)+pow(y_n,2);
            double tmp1=0;
            double tmp2=0;
            for(int k=0;k<max_dim+1;k++){
                tmp1+= ds[k]*pow(s_n,k);
                tmp2+= (2*k+1)*ds[k]*pow(s_n,k);
            }
            double Jdn = tmp1*tmp2;
            

            double x_d_t = tmp1*x_n;
            double y_d_t = tmp1*y_n;

            double J = Jdn*Jnw*Jwrt;

            // A_d += J*r_step*t_step*scale_i*scale_j/4;
            // x_d += x_d_t*J*r_step*t_step*scale_i*scale_j/4;
            // y_d += y_d_t*J*r_step*t_step*scale_i*scale_j/4;
            A_d += J*r_step*t_step;
            x_d += x_d_t*J*r_step*t_step;
            y_d += y_d_t*J*r_step*t_step;
        }
    }

    x_d = x_d/A_d;
    y_d = y_d/A_d;
    return Point(x_d, y_d);
}


Point MomentsTracker::ne2dp_Numerical(Eigen::Matrix3d Q, vector<double> ds){
    /*
    input: ellips in normal plane and distortion parameter
    output: centor point of region of distorted ellipse
    */

    int total_iter =100;
    int iter = (int)(sqrt(total_iter));
    array<double, 5> ellipse = ellipse2array(Q);
    

    double a,b,tx,ty,alpha;
    tx = ellipse[0];
    ty = ellipse[1];
    a = ellipse[2];
    b = ellipse[3];
    alpha = ellipse[4];
    double A_d=0; // =Ad/An
    double x_d=0;
    double y_d=0;

    double r_step = 1.0/iter;
    double t_step = 2*M_PI/iter; // t == theta

    // for(int i=0;i<iter+1;i++){
    //     for(int j=0;j<iter+1;j++){
    //         int scale_i=4;
    //         int scale_j=4;
    //         if(i==0 || i==iter) scale_i=1;
    //         else if( i%2==0) scale_i=2;
    //         if(j==0 || j==iter) scale_j=1;
    //         else if( j%2==0) scale_j=2;
    //         double r = r_step*(i);
    //         double t = t_step*(j);

    //         double x_n = tx+a*r*cos(alpha)*cos(t)-b*r*sin(alpha)*sin(t);
    //         double y_n = ty+a*r*sin(alpha)*cos(t)+b*r*cos(alpha)*sin(t);
    //         double s_n = pow(x_n,2)+pow(y_n,2);
    //         double tmp1=0;
    //         double tmp2=0;
    //         for(int k=0;k<max_dim+1;k++){
    //             tmp1+= ds[k]*pow(s_n,k);
    //             tmp2+= (2*k+1)*ds[k]*pow(s_n,k);
    //         }
    //         double Jdn = tmp1*tmp2;
    //         double Jnrt = a*b*r;

    //         double x_d_t = tmp1*x_n;
    //         double y_d_t = tmp1*y_n;

    //         A_d += Jdn*Jnrt*r_step*t_step*scale_i*scale_j/9;
    //         x_d += x_d_t*Jdn*Jnrt*r_step*t_step*scale_i*scale_j/9;
    //         y_d += y_d_t*Jdn*Jnrt*r_step*t_step*scale_i*scale_j/9;
    //     }
    // }

    for(int i=0;i<iter+1;i++){
        for(int j=0;j<iter+1;j++){
            int scale_i=2;
            int scale_j=2;
            if(i==0 || i==iter) scale_i=1;
            if(j==0 || j==iter) scale_j=1;
            double r = r_step*(i);
            double t = t_step*(j);

            double x_n = tx+a*r*cos(alpha)*cos(t)-b*r*sin(alpha)*sin(t);
            double y_n = ty+a*r*sin(alpha)*cos(t)+b*r*cos(alpha)*sin(t);
            double s_n = pow(x_n,2)+pow(y_n,2);
            double tmp1=0;
            double tmp2=0;
            for(int k=0;k<max_dim+1;k++){
                tmp1+= ds[k]*pow(s_n,k);
                tmp2+= (2*k+1)*ds[k]*pow(s_n,k);
            }
            double Jdn = tmp1*tmp2;
            double Jnrt = a*b*r;

            double x_d_t = tmp1*x_n;
            double y_d_t = tmp1*y_n;

            A_d += Jdn*Jnrt*r_step*t_step*scale_i*scale_j/4;
            x_d += x_d_t*Jdn*Jnrt*r_step*t_step*scale_i*scale_j/4;
            y_d += y_d_t*Jdn*Jnrt*r_step*t_step*scale_i*scale_j/4;
        }
    }
    // for(int i=0;i<iter;i++){
    //     for(int j=0;j<iter;j++){
    //         double r = r_step*(i+0.5);
    //         double t = t_step*(j+0.5);

    //         double x_n = tx+a*r*cos(alpha)*cos(t)-b*r*sin(alpha)*sin(t);
    //         double y_n = ty+a*r*sin(alpha)*cos(t)+b*r*cos(alpha)*sin(t);
    //         double s_n = pow(x_n,2)+pow(y_n,2);
    //         double tmp1=0;
    //         double tmp2=0;
    //         for(int k=0;k<max_dim+1;k++){
    //             tmp1+= ds[k]*pow(s_n,k);
    //             tmp2+= (2*k+1)*ds[k]*pow(s_n,k);
    //         }
    //         double Jdn = tmp1*tmp2;
    //         double Jnrt = a*b*r;

    //         double x_d_t = tmp1*x_n;
    //         double y_d_t = tmp1*y_n;

    //         A_d += Jdn*Jnrt*r_step*t_step;
    //         x_d += x_d_t*Jdn*Jnrt*r_step*t_step;
    //         y_d += y_d_t*Jdn*Jnrt*r_step*t_step;
    //     }
    // }
    x_d = x_d/A_d;
    y_d = y_d/A_d;
    return Point(x_d, y_d);
}


Point MomentsTracker::project(double wx, double wy, double r,Params intrinsic, Eigen::Matrix3d E, int mode){
    Eigen::Matrix3d Cw;
    Cw <<   1.0, 0.0, -wx,
            0.0, 1.0, -wy,
            -wx, -wy, pow(wx,2)+pow(wy,2)-pow(r,2);

    vector<double> ds = {1, intrinsic.d[0], intrinsic.d[1],intrinsic.d[2],intrinsic.d[3]};
    Point dp(0,0);
    if(mode ==0){
        Eigen::Matrix3d E_inv=  E.inverse();
        Eigen::Matrix3d Qn = E_inv.transpose()*Cw*E_inv;
        dp = ne2dp(Qn,ds);
    }
    else if(mode == 1){
        Eigen::Matrix3d E_inv=  E.inverse();
        Eigen::Matrix3d Qn = E_inv.transpose()*Cw*E_inv;
        array<double,5> ellipse_n= ellipse2array(Qn);
        Point pn(ellipse_n[0],ellipse_n[1]);
        dp = distort_Point(pn,ds);
    }
    else if(mode == 2){
        Eigen::Vector3d Pw{wx,wy,1};
        Eigen::Vector3d Pn = E*Pw;
        Point pn(Pn[0]/Pn[2],Pn[1]/Pn[2]);
        dp = distort_Point(pn,ds);
    }
    else if(mode == 3){
        Eigen::Matrix3d E_inv=  E.inverse();
        Eigen::Matrix3d Qn = E_inv.transpose()*Cw*E_inv;
        dp = ne2dp_Numerical(Qn,ds);
    }
    else if(mode == 4){
        dp = wc2dp_Numerical(Cw,E,ds);
    }
    else{
        throw MomentsTrackerError();
    }

    double u_e = dp.x*intrinsic.fx+dp.y*intrinsic.skew+intrinsic.cx; 
    double v_e = dp.y*intrinsic.fy+intrinsic.cy;

    return Point(u_e, v_e);
}