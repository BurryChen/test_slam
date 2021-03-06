#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

#include <dirent.h>//遍历系统指定目录下文件要包含的头文件
#include <iostream>
#include <sys/types.h>
#include <string>  
#include <vector>  
#include <fstream>  
#include <sstream> 
#include <unistd.h>

using namespace std;
using namespace cv;


void pose_estimation_3d3d (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R, Mat& t
);

void bundleAdjustment(
    const vector<Point3f>& points_3d,
    const vector<Point3f>& points_2d,
    Mat& R, Mat& t
);

// g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly( const Eigen::Vector3d& point ) : _point(point) {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        // measurement is p, point is p'
        _error = _measurement - pose->estimate().map( _point );
    }
    
    virtual void linearizeOplus()
    {
        g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Eigen::Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        
        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = -z;
        _jacobianOplusXi(0,2) = y;
        _jacobianOplusXi(0,3) = -1;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = 0;
        
        _jacobianOplusXi(1,0) = z;
        _jacobianOplusXi(1,1) = 0;
        _jacobianOplusXi(1,2) = -x;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -1;
        _jacobianOplusXi(1,5) = 0;
        
        _jacobianOplusXi(2,0) = -y;
        _jacobianOplusXi(2,1) = x;
        _jacobianOplusXi(2,2) = 0;
        _jacobianOplusXi(2,3) = 0;
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = -1;
    }

    bool read ( istream& in ) {}
    bool write ( ostream& out ) const {}
protected:
    Eigen::Vector3d _point;
};

int main ( int argc, char** argv )
{   
    vector<Point3f> pts1, pts2;
    int num=0;
   
   //string workdir="/media/whu/Research/04Research_PhD/01LRF_Calibation/data/linuxdata20180408/T2-L1-1-L2-1.bag_scan_ver";
   string workdir="/media/whu/HD_CHEN_2T/02data/WHUKylinBackpack/VLP16_Calibration/_2018-05-22-21-58-35.bag_pc_ver";
   chdir(workdir.c_str());
   ifstream inFile; 
   inFile.open("good_correspondences.csv",ios::in);
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
        //double R=0.315;
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
	//if(r1>R*0.707||r2>R*0.707)  continue;

	Point3f p1(x1,y1,z1);
        Point3f p2(x2,y2,z2);
        pts1.push_back ( p1);
        pts2.push_back ( p2); 
	num++;
        cout <<"corresponding-"<<num<< ":\t"<< x1 << "\t" << y1 << "\t" << z1 << "\t"<<x2<< "\t" << y2 << "\t" << z2 << endl; 
        outFile2 << num << ','  << x1<< ','<< y1 << ',' << z1<< ',' << x2<< ',' << y2 << ',' << z2 << endl;  
    }

    cout<<"3d-3d pairs: "<<pts1.size() <<endl;
    Mat R, t;
    pose_estimation_3d3d ( pts1, pts2, R, t );
    cout<<"ICP via SVD results: "<<endl;
    cout<<"R = "<<R<<endl;
    cout<<"t = "<<t<<endl;
    cout<<"R_inv = "<<R.t() <<endl;
    cout<<"t_inv = "<<-R.t() *t<<endl;

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

    bundleAdjustment( pts1, pts2, R, t );
    outFile<<" G2O bundleAdjustment results: "<<endl;
    outFile<<"R = "<<R<<endl;
    outFile<<"t = "<<t<<endl;
    outFile<<"R_inv = "<<R.t() <<endl;
    outFile<<"t_inv = "<<-R.t() *t<<endl;
    
    //2.Validation_residual: // verify p1 = R*p2 + t
    ofstream resFile;
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
    }
    
    
    return 1;
    
}


void pose_estimation_3d3d (
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

    // compute q1*q2^T
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

void bundleAdjustment (
    const vector< Point3f >& pts1,
    const vector< Point3f >& pts2,
    Mat& R, Mat& t )
{
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    pose->setId(0);
    pose->setEstimate( g2o::SE3Quat(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d( 0,0,0 )
    ) );
    optimizer.addVertex( pose );

    // edges
    int index = 1;
    vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly( 
            Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z) );
        edge->setId( index );
        edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
        edge->setMeasurement( Eigen::Vector3d( 
            pts1[i].x, pts1[i].y, pts1[i].z) );
        edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose( true );
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"optimization costs time: "<<time_used.count()<<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;
    
}
