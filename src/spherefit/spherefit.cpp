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
#include "Eigen/Dense"
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

typedef pcl::PointXYZ PointT;

typedef struct center {
  //us为单位
  long long stamp;   
  double x,y,z,r,rms;
}Center;

int circle2dfit(std::string infile,std::string outfile, pcl::ModelCoefficients::Ptr coefficients_circle2d,double *threshold)
{
  std::string fn;
  //fn=argv[1]; 
  //fn="table_scene_mug_stereo_textured.pcd";
  fn=infile;
  // All the objects needed
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::PLYWriter plywriter;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_circle2d (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  reader.read (fn, *cloud);  
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
  plywriter.write<PointT> (outfile+".ply", *cloud,false,false);

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (threshold[0], threshold[1]);
  pass.filter (*cloud_filtered);
   
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (threshold[2], threshold[3]);
  pass.filter (*cloud_filtered2);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered2->points.size () << " data points." << std::endl;
  
  if (cloud_filtered2->points.empty ()) 
  {
    std::cerr << "Can't find the cloud_filtered2 component." << std::endl;
    return 0;
  }
  else
  {
	  //std::cerr << "PointCloud representing the cloud_filtered2 component: " << cloud_filtered2->points.size () << " data points." << std::endl;
	  //writer.write ("cloud_filtered2.pcd", *cloud_filtered2, false);
          
	  plywriter.write<PointT> (outfile+"_filtered2.ply", *cloud_filtered2,false,false);
  }

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered2);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for circile2d segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CIRCLE2D);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0);
  seg.setMaxIterations (10);
  seg.setDistanceThreshold (0.005);
  seg.setRadiusLimits (0, 0.30);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals);

  // Obtain the circle2d inliers and coefficients
  seg.segment (*inliers_circle2d, *coefficients_circle2d);
  //std::cerr << "circle2d coefficients: " << *coefficients_circle2d << std::endl;

  // Write the circle2d inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_circle2d);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_circle2d (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_circle2d);
  if (cloud_circle2d->points.size()<50) 
  {
    std::cerr << "The circle2d poiny size less than 50." << std::endl;
    coefficients_circle2d->values.clear();
  } 
  else
  {
    //std::cerr << "PointCloud representing the circle2d component: " << cloud_circle2d->points.size () << " data points." << std::endl;
    //writer.write ("cloud_circle2d.pcd", *cloud_circle2d, false);
    plywriter.write<PointT> (outfile+"_circle2d.ply", *cloud_circle2d,false,false);
    // ransac precision-RMS
    vector<double> residual;
    for (size_t i = 0; i < cloud_circle2d->points.size (); ++i)
    {
      //center   
      double d=pow((cloud_circle2d->points[i].x -coefficients_circle2d->values[0]),2)
      +pow((cloud_circle2d->points[i].y -coefficients_circle2d->values[1]),2);
      d=sqrt(d)-coefficients_circle2d->values[2];   
      residual.push_back(d);
    } 
    double accum  = 0.0;  
    for (vector<int>::size_type ix = 0; ix != residual.size(); ix ++){
      accum  += residual[ix]*residual[ix];   
    }
    double rms = sqrt(accum/(residual.size()-1)); //RMS  
    coefficients_circle2d->values.push_back(rms);
  }

}

int transform(std::string infile,std::string outfile)
{
  // All the objects needed
  pcl::PCDReader reader;
  pcl::PLYWriter plywriter;
  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  // Read in the cloud data
  reader.read (infile, *cloud);  
 
  /* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
  */
  Eigen::Matrix4f transform= Eigen::Matrix4f::Identity();
  transform  <<   
-0.00112908,   -0.609369,    0.792886,   0.0341578,
  0.0466361,   -0.792055,   -0.608665,   -0.117896,
   0.998911,   0.0362899,   0.0293129,     -0.1441,
          0,           0,           0,           1;
	 
  // Print the transformation 
  printf ("Method: using a Matrix4f\n");
  //std::cout << transform << std::endl;
  
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
  plywriter.write<PointT> (outfile+"_transformed.ply", *transformed_cloud,false,false); 
}

int
main (int argc, char** argv)
{
  //1、定义变量
  vector<string> bag_dir;
  string workdir;
  DIR* pdir =NULL;
  dirent* p = NULL;
  ofstream outFile;
  vector <Center> centers_h,centers_v;
  char * tmp = new char[100];
  //double *threshold=new double[4];
  string subdir;
   
  // experiment 2  yoga
  double R=0.325;    
  bag_dir.push_back("/media/whu/Research/04Research_PhD/01LRF_Calibation/data/linuxdata20180408/T1-L1-0-L2-0.bag");
  bag_dir.push_back("/media/whu/Research/04Research_PhD/01LRF_Calibation/data/linuxdata20180408/T2-L1-1-L2-1.bag");
  //bag_dir.push_back("/media/whu/Research/LRF_Calibation/data/linuxdata20180408/T3-L1-1-L2-0.bag");
  //bag_dir.push_back("/media/whu/Research/LRF_Calibation/data/linuxdata20180408/T4-L1-1-L2-1.bag");
  int flag[8]={-1,-1,1,1,1,-1,1,1};
  double threshold[8][4]={
       0,1.0,   0,1.5,
    -0.3,0.3,-1.5,  0,    
       -1,0.3,  -1.7,0,
        0,0.5, 0, 1.5, 
       0,1.5,   0,1.5,
    -0.2,0.5,-2.0,  0,
       0,1.5,   0,1.5,
       0,0.5,-1.5,  0, 
  };
  
  for(int i= 0; i != bag_dir.size(); i ++)
  {
    //2、horizontial 圆心拟合及存储
    workdir=bag_dir[i]+"_scan_hori";
    //threshold[0]=0,threshold[1]=1.5,threshold[2]=0,threshold[3]=1.5;
    chdir(workdir.c_str());
    pdir = opendir(workdir.c_str()); 
    //清空子路径

    subdir=workdir+"/circle2d";
    //if(access(subdir.c_str(),F_OK)!=-1) rmdir(subdir.c_str());
    mkdir(subdir.c_str(),777);
    // 写文件     
    outFile.open("data_h.csv", ios::out);  
    outFile << "stamp" << ',' << "x" << ',' << "y" << ',' << "z" << ',' <<"r" << ',' << "rms" <<endl;  

    while((p = readdir(pdir)) != NULL)
    {
	//这里需要注意，linux平台下一个目录中有"."和".."隐藏文件，需要过滤掉
	//d_name是一个char数组，存放当前遍历到的文件名
	if(p->d_name[0] == 'h'&& strncmp(p->d_name + 18,".pcd", 5)==0)
	{
	  Center temp;	
	  strncpy(tmp, p->d_name + 2, 16); tmp[16] = '\0'; temp.stamp = atoll(tmp);   	
	  string temp1 =string(p->d_name);
	  string temp2= workdir+"/circle2d/"+string(p->d_name);	  
	    
	  cout<<temp1<<endl;
	  pcl::ModelCoefficients::Ptr coefficients_circle2d (new pcl::ModelCoefficients);
	  circle2dfit(temp1,temp2,coefficients_circle2d,threshold[2*i]);
	  if(coefficients_circle2d->values.size()==0) continue;
	  
	  temp.x=coefficients_circle2d->values[0];
	  temp.y=coefficients_circle2d->values[1];
	  temp.r=coefficients_circle2d->values[2];
	  temp.rms=coefficients_circle2d->values[3];
	  if(temp.r<R*0.707)	  temp.z=flag[i*2]*sqrt(R*R-temp.r*temp.r);
	  else continue;
	  centers_h.push_back(temp);
	  
	  cout<<temp.stamp << ',' << temp.x<< ',' << temp.y << ','<<temp.z << ',' << temp.r<< ',' << temp.rms<<endl<<endl;
	  outFile << temp.stamp << ',' << temp.x<< ',' << temp.y << ',' << temp.z << ','<<temp.r<< ',' << temp.rms<< endl;        
	}
    }
    
    closedir(pdir);  
    outFile.close();
      
    //3、vertical 圆心拟合及存储
    workdir=bag_dir[i]+"_scan_ver";
    //threshold[0]=-0.2,threshold[1]=0.5,threshold[2]=-1.5,threshold[3]=0;
    chdir(workdir.c_str());
    pdir = opendir(workdir.c_str()); 
    //清空子路径
    subdir=workdir+"/circle2d";
    //if(access(subdir.c_str(),F_OK)!=-1) rmdir(subdir.c_str());
    mkdir(subdir.c_str(),777);
    // 写文件     
    outFile.open("data_v.csv", ios::out);   
    outFile << "stamp" << ',' << "x" << ',' << "y" << ',' << "z" << ',' <<"r" << ',' << "rms" <<endl;  
    
    while((p = readdir(pdir)) != NULL)
    {
	//这里需要注意，linux平台下一个目录中有"."和".."隐藏文件，需要过滤掉
	//d_name是一个char数组，存放当前遍历到的文件名
	if(p->d_name[0] == 'v'&& strncmp(p->d_name + 18,".pcd", 5)==0)
	{
	  Center temp;	
	  strncpy(tmp, p->d_name + 2, 16); tmp[16] = '\0'; temp.stamp = atoll(tmp);   	
	  string temp1 =string(p->d_name);
	  string temp2= workdir+"/circle2d/"+string(p->d_name);	  
	  	    
	  cout<<temp1<<endl;
	  pcl::ModelCoefficients::Ptr coefficients_circle2d (new pcl::ModelCoefficients);
	  circle2dfit(temp1,temp2,coefficients_circle2d,threshold[2*i+1]);
	  if(coefficients_circle2d->values.size()==0) continue;
	  
	  temp.x=coefficients_circle2d->values[0];
	  temp.y=coefficients_circle2d->values[1];
	  temp.r=coefficients_circle2d->values[2];
	  temp.rms=coefficients_circle2d->values[3];	  
	  if(temp.r<R*0.707)	  temp.z=flag[i*2]*sqrt(R*R-temp.r*temp.r);
	  else continue;
	  centers_v.push_back(temp);
	  
	  cout<<temp.stamp << ',' << temp.x<< ',' << temp.y << ','<<temp.z << ',' << temp.r<< ',' << temp.rms<<endl<<endl;
	  outFile << temp.stamp << ',' << temp.x<< ',' << temp.y << ',' << temp.z << ','<<temp.r<< ',' << temp.rms<< endl; 	  
	}
    }
    
    closedir(pdir);  
    outFile.close();       
  }
  
  //4、时间戳对应
  // 写文件     
  outFile.open("data_vh.csv", ios::out);   
  outFile << "stamp1" << ',' << "x1" << ',' << "y1" << ',' << "z1" << ',' <<"r1" << ',' << "rms1" <<
  ',' << "stamp2" << ',' << "x2" << ',' << "y2" << ',' <<"z2" << ',' << "r2" << ',' <<
  { "rms2" <<endl;  
  for (vector<int>::size_type i= 0; i != centers_h.size(); i ++)
    for (vector<int>::size_type j= 0; j != centers_v.size(); j ++)
    {
      long long temp=abs(centers_h[i].stamp-centers_v[j].stamp);
      //40HZ,0.025s,
      if(temp<25000/2)
      {
	
	outFile << centers_h[i].stamp << ','  << centers_h[i].x<< ','<< centers_h[i].y << ',' << centers_h[i].z<< ',' 
	<< centers_h[i].r<< ',' << centers_h[i].rms
	<< ',' <<centers_v[j].stamp << ',' << centers_v[j].x<< ',' << centers_v[j].y << ',' << centers_v[j].z << ','
	<< centers_v[j].r<< ',' << centers_v[j].rms<< endl;  
      }	
    }
  }
  outFile.close();

   
   //5。 transform for validation
   bag_dir[0]="/media/whu/Research/04Research_PhD/01LRF_Calibation/data/linuxdata20180408/validation.bag";
   workdir=bag_dir[0]+"_scan_ver";
   //threshold[0]=-0.2,threshold[1]=0.5,threshold[2]=-1.5,threshold[3]=0;
   chdir(workdir.c_str());
   pdir = opendir(workdir.c_str()); 
   //清空子路径
   subdir=workdir+"/transform";
   mkdir(subdir.c_str(),777);
  
    while((p = readdir(pdir)) != NULL)
    {
	//这里需要注意，linux平台下一个目录中有"."和".."隐藏文件，需要过滤掉
	//d_name是一个char数组，存放当前遍历到的文件名
	if(p->d_name[0] == 'v'&& strncmp(p->d_name + 18,".pcd", 5)==0)
	{
	  string temp1 =string(p->d_name);
	  string temp3= workdir+"/transform/"+string(p->d_name);	
	  	  	 	  	  
	  //transform
	  transform(temp1,temp3);

	}
    }   
    closedir(pdir); 
   
  
  return (0);
}

