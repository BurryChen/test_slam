#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

using namespace std;
using namespace cv;

void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

// 像素坐标转相机归一化坐标
Point2d pixel2cam ( const Point2d& p, const Mat& K );

// 世界坐标转像素坐标
Point2d world2pixel ( const Point3d& p, const Mat& K, const Mat& R, const Mat& t);

void bundleAdjustment (
    const vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat& K,
    Mat& R, Mat& t
);

int main ( int argc, char** argv )
{
    /*if ( argc != 2 )
    {
        cout<<"usage: pose_estimation_2d3d corresponces"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    // 建立3D点
    Mat d1 = imread ( argv[3], CV_LOAD_IMAGE_UNCHANGED );       // 深度图为16位无符号数，单通道图像
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for ( DMatch m:matches )
    {
        ushort d = d1.ptr<unsigned short> (int ( keypoints_1[m.queryIdx].pt.y )) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        if ( d == 0 )   // bad depth
            continue;
        float dd = d/5000.0;
        Point2d p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, K );
        pts_3d.push_back ( Point3f ( p1.x*dd, p1.y*dd, dd ) );
        pts_2d.push_back ( keypoints_2[m.trainIdx].pt );
    }

    cout<<"3d-2d pairs: "<<pts_3d.size() <<endl;
       
    ofstream outFile;  
    outFile.open("cor_2d3d.csv", ios::out);   
    outFile  << "id"<<','<<"x" << ',' << "y" << ',' << "X" << ',' <<  "Y" << ',' << "Z" <<endl;
    for ( int i=0; i<pts_2d.size(); i++ )
    {
      //cout <<"corresponding-"<<i<< ":\t"<< pts_2d[i] << "\t" << pts_3d[i]  << endl; 
      outFile << i << ','  << pts_2d[i].x<< ','<< pts_2d[i].y << ',' << pts_3d[i].x<< ',' << pts_3d[i].y<< ',' << pts_3d[i].z  << endl;  
    }*/
    
    string workdir="/media/whu/Research/04SLAM_DoctoralDissertation/05InfraredCamera-LiDAR/01data/LiDAR_camera_calib";
    chdir(workdir.c_str());
   
    ifstream inFile; 
    inFile.open("corresponces_2d3d_01.csv",ios::in);
    string line; 
    //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取 ,第一行跳过
    getline(inFile, line);
    int id=0;
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    while (getline(inFile, line))   
    {  
        cout <<"原始字符串："<< line << endl; //整行输出  
        istringstream sin(line); //将整行字符串line读入到字符串流istringstream中  
        vector<string> fields; //声明一个字符串向量  
        string field;  
        while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符  
        {  
            fields.push_back(field); //将刚刚读取的字符串添加到向量fields中  
        }  
        float x=stof(fields[1].c_str()),y = atof(fields[2].c_str());
	float X=stof(fields[3].c_str()),Y = atof(fields[4].c_str()),Z = atof(fields[5].c_str());

	Point2f p1(x,y);
        Point3f p2(X,Y,Z);
        pts_2d.push_back ( p1);
        pts_3d.push_back ( p2); 
        cout <<"correspondence-"<<id++<<":\t"<< p1 << "\t" << p2  << endl; 
    }
    Mat K = ( Mat_<double> ( 3,3 ) << 2603.42438406452, 1.28877447628778, 1954.85741239866, 0, 2613.39945791598, 1463.05724922524, 0, 0, 1 );
    
    Mat r, t;
    solvePnP ( pts_3d, pts_2d, K, Mat(), r, t, false ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    Mat R;
    cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵

    cout<<"R="<<endl<<R<<endl;
    cout<<"t="<<endl<<t<<endl;

    cout<<"calling bundle adjustment"<<endl;

    //bundleAdjustment ( pts_3d, pts_2d, K, R, t );
    
    ofstream outFile;
    outFile.open("result.log", ios::out);  
    outFile<<" G2O bundleAdjustment results: "<<endl;
    outFile<<"R = "<<R<<endl;
    outFile<<"t = "<<t<<endl;
    outFile<<"R_inv = "<<R.t() <<endl;
    outFile<<"t_inv = "<<-R.t() *t<<endl;
    
    //2.Validation_residual: // verify pts_2d = R*p2 + t
    ofstream resFile;
    resFile.open("residual.csv", ios::out);     
    for ( int i=0; i<pts_2d.size(); i++ )
    {
        Point2d pts_3d2pixel=world2pixel(pts_3d[i],K,R,t);
	Point2d error_proj(pts_2d[i].x-pts_3d2pixel.x ,pts_2d[i].y-pts_3d2pixel.y);
        
	cout<<"pts_2d = "<<pts_2d[i]<<endl;
        cout<<"pts_3d = "<<pts_3d[i]<<endl;
        cout<<"pts_3d2pixel = "<< pts_3d2pixel <<endl;	    
	cout<<"error_proj = "<< error_proj<<endl;
        cout<<endl;
	
	outFile<<"pts_2d = "<<pts_2d[i]<<endl;
        outFile<<"pts_3d = "<<pts_3d[i]<<endl;
        outFile<<"pts_3d2pixel = "<< pts_3d2pixel <<endl;	    
	outFile<<"error_proj = "<< error_proj<<endl;
        outFile<<endl;
	
	resFile<<
	//pts_2d[i]<<pts_3d[i]<<pts_3d2pixel<< 
	error_proj.x<<","<<error_proj.y<<","<<norm(error_proj)<<endl;
    }
}

void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

// 世界坐标转像素坐标
Point2d world2pixel ( const Point3d& p, const Mat& K, const Mat& R, const Mat& t)
{
  Mat KTP=K*(R * (Mat_<double>(3,1)<<p.x, p.y, p.z) + t);
  return Point2d(KTP.at<double>(0,0) / KTP.at<double>(0,2),KTP.at<double>(0,1) / KTP.at<double>(0,2));
}

void bundleAdjustment (
    const vector< Point3f > points_3d,
    const vector< Point2f > points_2d,
    const Mat& K,
    Mat& R, Mat& t )
{
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                        ) );
    optimizer.addVertex ( pose );

    int index = 1;
    for ( const Point3f p:points_3d )   // landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId ( index++ );
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        optimizer.addVertex ( point );
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // edges
    index = 1;
    for ( const Point2f p:points_2d )
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;
}
