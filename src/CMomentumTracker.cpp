#include "CMomentumTracker.h"

MomentumTracker::MomentumTracker(int dim)
{
    this-> max_dim = dim;
    this->max_n = max_dim*3;
    this->comb_max = max_n*2+1;
    init();
}

MomentumTracker::MomentumTracker(){
    this-> max_dim = 4;
    this->max_n = max_dim*3;
    this->comb_max = max_n*2+1;
    init();
}
MomentumTracker::~MomentumTracker()
{
}

void MomentumTracker::init(){
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

double MomentumTracker::nCr(int n, int r){
    if(n> comb_max || n<r) throw MomentumTrackerError();
    return (double)(comb_buffer[n][r]);
}
double MomentumTracker::_M_2i2j(int i, int j){
    // a, b == 1
    double den= nCr(2*(i+j),i+j)*nCr(i+j,i);
    double num= nCr(2*(i+j),2*i)*(i+j+1)*pow(2,2*(i+j));
    // cout<<den<<num<<endl;
    return den/num;
    
}
double MomentumTracker::M_2i2j(int i, int j){
    if(i> max_n || j>max_n) throw MomentumTrackerError();
    else return moment_buffer[i][j];
    
}
double MomentumTracker::M_2i2j(int i, int j, double a, double b){
    // return M^{2i2j}
    return M_2i2j(i,j)*pow(a,2*i)*pow(b,2*j);
}


array<double ,3> MomentumTracker::intSn(int n,double a, double b, double tx, double ty, double alpha){
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
double MomentumTracker::C0n(int n,vector<double> ds){
    double result=0;
    int n_d =max_dim;
    for(int i=max(0,n-n_d);i<min(n,n_d)+1;i++){
        result+= (2*i+1)*ds[i]*ds[n-i];
    }
    return result;
}
double MomentumTracker::C1n(int n,vector<double> ds){
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

array<double, 5> MomentumTracker::ellipse2array(Eigen::Matrix3d Q){
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

Point MomentumTracker::ne2dp(Eigen::Matrix3d Q, vector<double> ds){
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
    // a=0;
    // b=0;
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

Point MomentumTracker::distort_Point(Point pn, vector<double> ds){
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

//-------------static----------------//

// long long int MomentumTracker::fact(int n){
//     return fact(n,n);
// }

// long long int MomentumTracker::fact(int n, int k){
//     // n>=k
//     if(k==0) return 1;
//     if (k>0) return n*fact(n-1,k-1);
// }

// int MomentumTracker::_nCr(int n,int r){
//     if(n<r) return 0;
//     if(n==r) return 1;
//     if (r==0&&n!=0) return 1;

//     if( r > n/2){
//         return _nCr(n,n-r);
//     }
//     else return fact(n,r)/fact(r);
// }