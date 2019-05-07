#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

#include <dirent.h>//遍历系统指定目录下文件要包含的头文件
#include <iostream>
#include <sys/types.h>
#include <string>  
#include <vector>  
#include <fstream>  
#include <sstream> 
#include <unistd.h>
using namespace std;

using namespace pcl;

#define PI 3.14159262728

typedef pcl::PointXYZ PointT;

void
lineToLineSegment (const Eigen::VectorXf &line_a, const Eigen::VectorXf &line_b, 
                        Eigen::Vector4f &pt1_seg, Eigen::Vector4f &pt2_seg)
{
  // point + direction = 2nd point
  Eigen::Vector4f p1 = Eigen::Vector4f::Zero ();
  Eigen::Vector4f p2 = Eigen::Vector4f::Zero ();
  Eigen::Vector4f dir1 = Eigen::Vector4f::Zero ();
  p1.head<3> () = line_a.head<3> ();
  dir1.head<3> () = line_a.segment<3> (3);
  p2 = p1 + dir1;

  // point + direction = 2nd point
  Eigen::Vector4f q1 = Eigen::Vector4f::Zero ();
  Eigen::Vector4f q2 = Eigen::Vector4f::Zero ();
  Eigen::Vector4f dir2 = Eigen::Vector4f::Zero ();
  q1.head<3> () = line_b.head<3> ();
  dir2.head<3> () = line_b.segment<3> (3);
  q2 = q1 + dir2;

  // a = x2 - x1 = line_a[1] - line_a[0]
  Eigen::Vector4f u = dir1;
  // b = x4 - x3 = line_b[1] - line_b[0]
  Eigen::Vector4f v = dir2;
  // c = x2 - x3 = line_a[1] - line_b[0]
  Eigen::Vector4f w = p2 - q1;

  float a = u.dot (u);
  float b = u.dot (v);
  float c = v.dot (v);
  float d = u.dot (w);
  float e = v.dot (w);
  float denominator = a*c - b*b;
  float sc, tc;
  // Compute the line parameters of the two closest points
  if (denominator < 1e-5)          // The lines are almost parallel
  {
    sc = 0.0;
    tc = (b > c ? d / b : e / c);  // Use the largest denominator
  }
  else
  {
    sc = (b*e - c*d) / denominator;
    tc = (a*e - b*d) / denominator;
  }
  // Get the closest points
  pt1_seg = Eigen::Vector4f::Zero ();
  pt1_seg = p2 + sc * u;

  pt2_seg = Eigen::Vector4f::Zero ();
  pt2_seg = q1 + tc * v;
}

double 
lineWithLineIntersection (const Eigen::VectorXf &line_a, 
                               const Eigen::VectorXf &line_b, 
                               Eigen::Vector4f &point, double sqr_eps)
{
  Eigen::Vector4f p1, p2;
  lineToLineSegment (line_a, line_b, p1, p2);

  // If the segment size is smaller than a pre-given epsilon...
  double sqr_dist = (p1 - p2).squaredNorm ();
  if (sqr_dist < sqr_eps)
  {
    point = p1;
    return sqr_dist;
  }
  point.setZero ();
  return sqr_dist;
}

double 
lineWithLineIntersection (const pcl::ModelCoefficients &line_a, 
                               const pcl::ModelCoefficients &line_b, 
                               Eigen::Vector4f &point, double sqr_eps)
{
  Eigen::VectorXf coeff1 = Eigen::VectorXf::Map (&line_a.values[0], line_a.values.size ());
  Eigen::VectorXf coeff2 = Eigen::VectorXf::Map (&line_b.values[0], line_b.values.size ());
  return (lineWithLineIntersection (coeff1, coeff2, point, sqr_eps));
}

int
main (int argc, char** argv)
{
  string workdir="/media/whu/Research/04Dissertation/05Lidar2Camera_Calib/01data/l2v_calib03_20190415_vlp16/train/cps";
  //string workdir=argv[1];
  chdir(workdir.c_str());
  opendir(workdir.c_str());
   
  //1.read break_2d from Infrared Image
  vector<Eigen::Vector3f> breaks_2d;
  ifstream inFile;
  inFile.open("break_2d.csv",ios::in);
  string line; 
  while (getline(inFile, line))   
  {  
    //cout <<"原始字符串："<< line << endl; //整行输出  
    istringstream sin(line); //将整行字符串line读入到字符串流istringstream中  
    vector<string> fields; //声明一个字符串向量  
    string field;  
    while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符  
    {
      fields.push_back(field); //将刚刚读取的字符串添加到向量fields中  
    }  
    Eigen::Vector3f p_2d;
    p_2d[0]=atof(fields[0].c_str()),p_2d[1] = atof(fields[1].c_str());p_2d[2] = atof(fields[2].c_str());
    breaks_2d.push_back(p_2d);
   }
   inFile.close();
  
  //2. break_3d extract
  vector<Eigen::Vector4f> breaks_3d,precisons;
  Eigen::Vector4f precison;
  ofstream outFile;

  char * tmp = new char[100];
  int break_id=0;
  struct dirent **namelist;
  dirent* p = NULL;
  int n=scandir(workdir.c_str(),&namelist,0,alphasort);
  if(n < 0) cout << "scandir return "<< n  << endl;
    
  int index=0;
  while(index < n)
  {
    //这里需要注意，linux平台下一个目录中有"."和".."隐藏文件，需要过滤掉
    //d_name是一个char数组，存放当前遍历到的文件名
    p=namelist[index];
    //printf("%s\n", namelist[index]->d_name);
    if(p->d_name[0] == 'b'&& strncmp(p->d_name + 15,".pcd", 5)==0)
    {
  std::string input_pcd =string(namelist[index]->d_name);
  strncpy(tmp, p->d_name+11, 4); tmp[4] = '\0';break_id=atol(tmp);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());

  if(pcl::io::loadPCDFile(input_pcd, *cloud_src)) {
    std::cerr << "failed to load " << input_pcd << std::endl;
    return 0;
  }
  //std::cout<<"input_pcd size:"<<cloud_src->size()<<std::endl;
  
  //1. line segmentation detection
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients coefficients1,coefficients2;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_line (new pcl::PointCloud<pcl::PointXYZI>);
  
  seg.setOptimizeCoefficients (true);                       //设置优化参数
  seg.setModelType (pcl::SACMODEL_LINE);                    //设置分割模型为直线
  seg.setMethodType (pcl::SAC_RANSAC);                      //参数估计方法
  //seg.setMaxIterations (100);                             //最大迭代次数
  seg.setDistanceThreshold (0.005);                         //内点到模型的距离  
  
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  std::string ss;
  //Line1 
  seg.setInputCloud (cloud_src);
  seg.segment (*inliers, coefficients1);
  
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.setInputCloud (cloud_src);
  extract.filter (*cloud_line);
  extract.setNegative (true);
  extract.filter (*cloud_temp);    
  *cloud_src = *cloud_temp;  
  ss = "line_"+string(tmp)+"_L1.pcd";
  //std::cout << ss <<" "<<cloud_line->points.size()<<coefficients1<< std::endl;
  //pcl::io::savePCDFileASCII (ss, *cloud_line);
  // ransac precision-RMS
  vector<double> residual;
  for (size_t i = 0; i < cloud_line->points.size (); ++i)
    {
      Vector3d v1,a,d;
      v1<<cloud_line->points[i].x -coefficients1.values[0],
      cloud_line->points[i].y -coefficients1.values[1],
      cloud_line->points[i].z -coefficients1.values[2];
      a<<coefficients1.values[3],coefficients1.values[4],coefficients1.values[5];
      d=v1.cross(a);
      residual.push_back(d.norm());
    } 
  double accum  = 0.0;  
  for (vector<int>::size_type ix = 0; ix != residual.size(); ix ++){
      accum  += residual[ix]*residual[ix];   
    }
  precison[0]= sqrt(accum/(residual.size()-1)); //RMS  

    
  //Line2
  seg.setInputCloud (cloud_src);
  seg.segment (*inliers, coefficients2);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.setInputCloud (cloud_src);
  extract.filter (*cloud_line);
  extract.setNegative (true);
  extract.filter (*cloud_temp);   
  *cloud_src = *cloud_temp; 
  ss ="line_"+string(tmp)+"_L2.pcd";
  //std::cout << ss <<" "<<cloud_line->points.size()<<coefficients2<< std::endl;
  //pcl::io::savePCDFileASCII (ss, *cloud_line);
    // ransac precision-RMS
  residual.clear();
  for (size_t i = 0; i < cloud_line->points.size (); ++i)
    {
      Vector3d v1,a,d;
      v1<<cloud_line->points[i].x -coefficients2.values[0],
      cloud_line->points[i].y -coefficients2.values[1],
      cloud_line->points[i].z -coefficients2.values[2];
      a<<coefficients2.values[3],coefficients2.values[4],coefficients2.values[5];
      d=v1.cross(a);
      residual.push_back(d.norm());
    } 
  accum  = 0.0;  
  for (vector<int>::size_type ix = 0; ix != residual.size(); ix ++){
      accum  += residual[ix]*residual[ix];   
    }
  precison[1]= sqrt(accum/(residual.size()-1)); //RMS  
  
  //2.lineWithLineIntersection
  Eigen::Vector4f break_3d;
  double sqr_eps=1e-6;
  precison[2]=lineWithLineIntersection(coefficients1,coefficients2,break_3d,sqr_eps);
  break_3d[3]=break_id;
  precison[3]=break_id;
  std::cout<<break_id<<","<<break_3d[0]<<","<<break_3d[1]<<","<<break_3d[2]<< std::endl;
  if(break_3d[0]!=0){
    breaks_3d.push_back(break_3d);
    precisons.push_back(precison);
  }

	}
	free(namelist[index]);
	index++;
  }
  free(tmp);
 
  // write corresponces_2d3d.csv  
  outFile.open("corresponces_2d3d.csv", ios::out);  
  //outFile << "id"<< ',' <<"u" << ',' << "v"  << ',' << "X" << ',' << "X" << ',' << "Z" <<endl; 
  for(int i=0;i<breaks_2d.size();i++)
  {
      for(int j=0;j<breaks_3d.size();j++)
      {
	if(breaks_2d[i][2]==breaks_3d[j][3]){
          //std::cout<<i<<","<<breaks_2d[i][0]<<","<<breaks_2d[i][1]<<","<<breaks_3d[j][0]<<","<<breaks_3d[j][1]<<","<<breaks_3d[j][2]<<","<<breaks_3d[j][3]<< std::endl;
          outFile  <<i<<","<<breaks_2d[i][0]<<","<<breaks_2d[i][1]<<","<<breaks_3d[j][0]<<","<<breaks_3d[j][1]<<","<<breaks_3d[j][2]<<","<<breaks_3d[j][3]
          <<","<<precisons[j][0]<<","<<precisons[j][1]<<","<<precisons[j][2]<<","<<precisons[j][3]<< std::endl;;	  
	}
      }
  }
  outFile.close();
  
  return (0);
}

