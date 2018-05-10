#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <chrono>


#include <dirent.h>//遍历系统指定目录下文件要包含的头文件
#include <iostream>
#include <sys/types.h>
#include <string>  
#include <vector>  
#include <fstream>  
#include <sstream> 
#include <unistd.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace std;
using namespace cv;


void pose_estimation_3d3d_svd (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R, Mat& t
);


void bundleAdjustment_onlypose_ceres(
    const vector< Point3f >& pts1,
    const vector< Point3f >& pts2,
    Mat& R, Mat& t
);

void bundleAdjustment_posesphere_ceres(
    const vector< Point3f >& pts1,
    const vector< Point3f >& pts2,
    Mat& R, Mat& t
);


int main ( int argc, char** argv )
{   
    vector<Point3f> pts1, pts2;
    int num=0;
   
   string workdir="/media/whu/Research/01LRF_Calibation/data/linuxdata20180408/T2-L1-1-L2-1.bag_scan_ver";
   chdir(workdir.c_str());
   ifstream inFile;
   inFile.open("data_vh.csv",ios::in);
   ofstream outFile,outFile2;
   outFile.open("result.log", ios::out);  
   
   outFile2.open("data_vh_seleted.csv", ios::out);   
   outFile2  << "num"<<','<<"x1" << ',' << "y1" << ',' << "z1" << ',' <<  "x2" << ',' << "y2" << ',' <<"z2"<<endl;  
 
   string line; 
   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取 ,第一行跳过
    getline(inFile, line);
    num=0;
    while (getline(inFile, line))   
    {  
        double R=0.315;
        //cout <<"原始字符串："<< line << endl; //整行输出  
        istringstream sin(line); //将整行字符串line读入到字符串流istringstream中  
        vector<string> fields; //声明一个字符串向量  
        string field;  
        while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符  
        {  
            fields.push_back(field); //将刚刚读取的字符串添加到向量fields中  
        }  
        double x1=stof(fields[1].c_str()),y1 = atof(fields[2].c_str()),z1 = atof(fields[3].c_str()),r1 = atof(fields[4].c_str());
	double x2=stof(fields[7].c_str()),y2 = atof(fields[8].c_str()),z2 = atof(fields[9].c_str()),r2 = atof(fields[10].c_str());
	if(r1>R*0.707||r2>R*0.707)  continue;

	Point3f p1(x1,y1,z1);
        Point3f p2(x2,y2,z2);
        pts1.push_back ( p1);
        pts2.push_back ( p2); 
	num++;
        //cout <<"corresponding-"<<num<< ":\t"<< x1 << "\t" << y1 << "\t" << z1 << "\t"<<x2<< "\t" << y2 << "\t" << z2 << endl; 
        outFile2 << num << ','  << x1<< ','<< y1 << ',' << z1<< ',' << x2<< ',' << y2 << ',' << z2 << endl;  
    }

    cout<<"3d-3d pairs: "<<pts1.size() <<endl;
    Mat R, t;
    /*pose_estimation_3d3d_svd ( pts1, pts2, R, t );
    cout<<"ICP via SVD results: "<<endl;
    cout<<"R = "<<R<<endl;
    cout<<"t = "<<t<<endl;
    cout<<"R_inv = "<<R.t() <<endl;
    cout<<"t_inv = "<<-R.t() *t<<endl;*/

    // verify p1 = R*p2 + t
    /*for ( int i=0; i<pts1.size(); i++ )
    {
        cout<<"p1 = "<<pts1[i]<<endl;
        cout<<"p2 = "<<pts2[i]<<endl;
        cout<<"(R*p2+t) = "<< 
            R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t
            <<endl;	    
	cout<<"(R*p2+t-p1) = "<< 
           R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t - (Mat_<double>(3,1)<<pts1[i].x, pts1[i].y, pts1[i].z)
            <<endl;
        cout<<endl;
    }*/
    
    
    cout<<"calling bundle adjustment"<<endl;

    //bundleAdjustment_g2o( pts1, pts2, R, t );
    bundleAdjustment_onlypose_ceres( pts1, pts2, R, t );
    bundleAdjustment_posesphere_ceres( pts1, pts2, R, t );
    cout<<" ceres residual bundleAdjustment results: "<<endl;
    cout<<"R = "<<R<<endl;
    cout<<"t = "<<t<<endl;
    outFile<<" ceres residual bundleAdjustment results: "<<endl;
    outFile<<"R = "<<R<<endl;
    outFile<<"t = "<<t<<endl;
    outFile<<"R_inv = "<<R.t() <<endl;
    outFile<<"t_inv = "<<-R.t() *t<<endl;
    
    //2.Validation_residual: // verify p1 = R*p2 + t
    /*ofstream resFile;
    resFile.open("residual.csv", ios::out);     
    for ( int i=0; i<pts1.size(); i++ )
    {
        cout<<"p1 = "<<pts1[i]<<endl;
        cout<<"p2 = "<<pts2[i]<<endl;
        cout<<"(R*p2+t) = "<< 
            R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t
            <<endl;	    
	cout<<"(R*p2+t-p1) = "<< 
           R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t - (Mat_<double>(3,1)<<pts1[i].x, pts1[i].y, pts1[i].z)
            <<endl;
        cout<<endl;
	
	outFile<<"p1 = "<<pts1[i]<<endl;
        outFile<<"p2 = "<<pts2[i]<<endl;
        outFile<<"(R*p2+t) = "<< 
            R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t
            <<endl;	    
	outFile<<"(R*p2+t-p1) = "<< 
           R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t - (Mat_<double>(3,1)<<pts1[i].x, pts1[i].y, pts1[i].z)
            <<endl;
        outFile<<endl;
	
	Mat temp=R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t - (Mat_<double>(3,1)<<pts1[i].x, pts1[i].y, pts1[i].z);
	double Eucd=sqrt(temp.at<double>(0,0)*temp.at<double>(0,0)+temp.at<double>(1,0)*temp.at<double>(1,0)
	+temp.at<double>(2,0)*temp.at<double>(2,0));
	resFile<<//pts1[i]<<pts2[i]<< R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t<<
           temp.at<double>(0,0)<<","<<temp.at<double>(1,0)<<","<<temp.at<double>(2,0)<<","<<Eucd<<endl;
    }*/
    
    return 1;   
}


void pose_estimation_3d3d_svd (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R, Mat& t
)
{
    Point3f p1, p2;     // center of mass
    int N = pts1.size();
    for ( int i=0; i<N; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f( Vec3f(p1) /  N);
    p2 = Point3f( Vec3f(p2) / N);
    vector<Point3f>     q1 ( N ), q2 ( N ); // remove the center
    for ( int i=0; i<N; i++ )
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^Tpose_estimation_3d3d
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
    cout<<"W="<<W<<endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout<<"U="<<U<<endl;
    cout<<"V="<<V<<endl;

    Eigen::Matrix3d R_ = U* ( V.transpose() );
    Eigen::Vector3d t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );

    // convert to cv::Mat
    R = ( Mat_<double> ( 3,3 ) <<
          R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
          R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
          R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
        );
    t = ( Mat_<double> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) );
}


// 代价函数,res=T*O1-O2
// 已知量：O1(x1,y1,z1);O2(x2,y2,z2);
// 待估参数：pose
struct CORRESPONDING_ONLYPOSE_COST
{
    CORRESPONDING_ONLYPOSE_COST (double u1,double v1,double d1,
			double u2,double v2,double d2) 
    : _x1(u1),_y1(v1),_z1(d1),_x2(u2),_y2(v2),_z2(d2) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const pose,     // 模型参数，有6维
        T* residual ) const     //  残差   3维
    {	
        //RES;
	T O2[3],TO2[3];
	O2[0]=T(_x2),O2[1]=T(_y2),O2[2]=T(_z2);
        // Rodrigues' formula
        ceres::AngleAxisRotatePoint(pose, O2, TO2);
        // pose[3,4,5] are the translation
        TO2[0] += pose[3]; TO2[1] += pose[4]; TO2[2] += pose[5];	
	residual[0] = T(_x1) - TO2[0];
        residual[1] = T(_y1) - TO2[1];
	residual[2] = T(_z1) - TO2[2];
        return true;
    }
    const double _x1,_y1,_z1,_x2,_y2,_z2;    
};


// 代价函数,

// 已知量：p1(u1,v1,0),d1;p2(u2,v2,0),d2;
// 待估参数：pose+球心z坐标
struct CORRESPONDING_POSE_SPHERE_COST
{
    CORRESPONDING_POSE_SPHERE_COST (double u1,double v1,double d1,
			double u2,double v2,double d2) 
    : _x1(u1),_y1(v1),_d1(d1),_x2(u2),_y2(v2),_d2(d2) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const pose,     // pos参数，6维
	const T* const O1,       // O1坐标,3纬
	const T* const O2,       // O2坐标,3纬
        T* residual ) const      // 残差   2维
    {	
	T TO2[3];
	//O1[0]=T(_x1),O1[1]=T(_y1);O1[2]=T(_d1);
	//O2[0]=T(_x2),O2[1]=T(_y2);O2[2]=T(_d2);
        // Rodrigues' formula p=T*p2
        ceres::AngleAxisRotatePoint(pose, O2, TO2);
        // pose[3,4,5] are the translation
        TO2[0] += pose[3]; TO2[1] += pose[4]; TO2[2] += pose[5];
	//
	residual[0] = O1[0] - TO2[0];
        residual[1] = O1[1]-  TO2[1];
	residual[2] = O1[2] - TO2[2];
        residual[3] = O1[0] - T(_x1);
        residual[4] = O1[1]-  T(_y1);
	residual[5] = O1[2] - T(_d1);
	residual[6] = O2[0] - T(_x2);
        residual[7] = O2[1]-  T(_y2);
	residual[8] = O2[2] - T(_d2);
        return true;
    }
    const double _x1,_y1,_d1,_x2,_y2,_d2;    
};

void bundleAdjustment_onlypose_ceres (
    const vector< Point3f >& pts1,
    const vector< Point3f >& pts2,
    Mat& R, Mat& t )
{
    int num=pts1.size();   
    double pose[6]={0,0,0,0,0,0};   //estimation pose,先欧拉角后平移
    //build problem
    ceres::Problem problem;
    for ( int i=0; i<pts1.size(); i++ )
    {
        double p1[3],p2[3];
	p1[0]=pts1[i].x;p1[1]=pts1[i].y;p1[2]=pts1[i].z;
	p2[0]=pts2[i].x;p2[1]=pts2[i].y;p2[2]=pts2[i].z;
	//代价函数,优化参数pose
        problem.AddResidualBlock (     // 向问题中添加误差项
        // 使用自动求导，模板参数：误差类型，误差项维度，优化参数维度，维数要与前面struct中一致
            new ceres::AutoDiffCostFunction<CORRESPONDING_ONLYPOSE_COST, 3, 6> ( 
                new CORRESPONDING_ONLYPOSE_COST (p1[0],p1[1],p1[2],p2[0],p2[1],p2[2])
            ),
            nullptr,            // 核函数，这里不使用，为空
            pose                 // 待估计参数
        );	
    }
            
    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve ( options, &problem, &summary );  // 开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    // 输出结果
    cout<<summary.FullReport() <<endl;
    cout<<"estimated pose= ";
    for(auto a:pose) cout<<a<<" ";
    cout<<endl;
    
    double angle[3]={pose[0],pose[1],pose[2]};
    double I[3][3]={1,0,0,0,1,0,0,0,1},R_[3][3];
    // Rodrigues' formula
    ceres::AngleAxisRotatePoint(pose, I[0], R_[0]); 
    ceres::AngleAxisRotatePoint(pose, I[1], R_[1]); 
    ceres::AngleAxisRotatePoint(pose, I[2], R_[2]);
    // convert to cv::Mat
    R = ( Mat_<double> ( 3,3 ) <<
          R_[0][0],R_[1][0],R_[2][0],
	  R_[0][1],R_[1][1],R_[2][1],
	  R_[0][2],R_[1][2],R_[2][2]
        );
    t = ( Mat_<double> ( 3,1 ) << pose[3],pose[4],pose[5] );   

}

void bundleAdjustment_posesphere_ceres (
    const vector< Point3f >& pts1,
    const vector< Point3f >& pts2,
    Mat& R, Mat& t )
{
    int num=pts1.size();
    double pose0[6] = {1.8705,-0.596254,1.7975,0.0278155,-0.112934,-0.143136};   
    double pose[6]={0,0,0,0,0,0};   //estimation pose,先欧拉角后平移


    double O1[3*178]={0},O2[3*178]={0};//{1.8705,-0.596254,1.7975,0.0278155,-0.112934,-0.143136};
    ceres::Problem problem;
    for ( int i=0; i<num; i++ )
    {
        ceres::LossFunction* loss_function =new ceres::HuberLoss(1.0);
        double p1[2],p2[2],distance1,distance2;
	p1[0]=pts1[i].x;p1[1]=pts1[i].y;p1[2]=pts1[i].z;
	p2[0]=pts2[i].x;p2[1]=pts2[i].y;p2[2]=pts2[i].z;
	O1[3*i+0]=pts1[i].x;O1[3*i+1]=pts1[i].y;O1[3*i+2]=pts1[i].z;
	O2[3*i+0]=pts2[i].x;O2[3*i+1]=pts2[i].y;O2[3*i+2]=pts2[i].z;
	//代价函数,优化参数pose+球心z坐标
	// 使用自动求导，模板参数：误差类型，误差项维度，优化参数维度，维数要与前面struct中一致
	// make CostFunction
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<CORRESPONDING_POSE_SPHERE_COST,9,6,3,3> ( 
                new CORRESPONDING_POSE_SPHERE_COST (p1[0],p1[1],p1[2],p2[0],p2[1],p2[2]));
        problem.AddResidualBlock(cost_function,   
				 nullptr,
				 //new ceres::HuberLoss(1.0) ,            // 核函数，这里不使用，为空
                                 pose,              // 待估计参数 pose
	                         &(O1[3*i+0]),
				 &(O2[3*i+0])       // 待估计参数 spherez);
		);
    }
        
    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout
    
    ceres::Solver::Summary summary;                // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve ( options, &problem, &summary );  // 开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    // 输出结果
    cout<<summary.FullReport() <<endl;
    //cout<<summary.<<endl;
    cout<<"estimated pose= ";
    for(auto a:pose) cout<<a<<" ";
    cout<<endl;
    cout<<"estimated pose0= ";
    for(auto a:pose0) cout<<a<<" ";
    cout<<endl; 
    cout<<"estimated pose erroe = ";
    for ( int i=0; i<6; i++ ) cout<<pose[i]-pose0[i]<<" ";
    cout<<endl;   
        
    /*cout<<"estimated O-P = ";
    for ( int i=0; i<num; i++ ) 
    {
      cout<<O1[3*i+0]-pts1[i].x<<" "<<O1[3*i+1]-pts1[i].y<<" "<<O1[3*i+2]-pts1[i].z
      <<O2[3*i+0]-pts2[i].x<<" "<<O2[3*i+1]-pts2[i].y<<" "<<O2[3*i+2]-pts2[i].z<<endl;
    }
    cout<<endl;*/
    
    double angle[3]={pose[0],pose[1],pose[2]};
    double I[3][3]={1,0,0,0,1,0,0,0,1},R_[3][3];
    // Rodrigues' formula
    ceres::AngleAxisRotatePoint(pose, I[0], R_[0]); 
    ceres::AngleAxisRotatePoint(pose, I[1], R_[1]); 
    ceres::AngleAxisRotatePoint(pose, I[2], R_[2]);
    // convert to cv::Mat
    R = ( Mat_<double> ( 3,3 ) <<
          R_[0][0],R_[1][0],R_[2][0],
	  R_[0][1],R_[1][1],R_[2][1],
	  R_[0][2],R_[1][2],R_[2][2]
        );
    t = ( Mat_<double> ( 3,1 ) << pose[3],pose[4],pose[5] );   
}