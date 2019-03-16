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
#include <Eigen/Core>
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

#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/distances.h>
using namespace pcl;
using namespace pcl::registration;

#define PI 3.14159262728
#define Sign(y) (y>=0?1:-1)

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
  //pcl::PCDReader reader;
  pcl::PLYReader reader;
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
  //dataset1_linuxdata20180408
/*  transform  <<   
0.01196550552872411, -0.6113692088585728, 0.7912550266109437,   0.03253560139167337,
0.04014185027972883, -0.7903800869836213, -0.6113002126254196,   -0.1172873900636409,
0.9991223441268654, 0.03907695688517882, 0.01508419394704152,    -0.1448743537651672,
          0,           0,           0,           1;*/
// dataset9_angle90_0_20180621
  transform  <<   
  -0.02094681801281828, -0.07978796441100858, 0.9965917476832153, 0.05958551769557144,
 0.002419209174023197, -0.9968115833032846, -0.07975471659639455, -0.01636101075184244,
 0.9997776644044967, 0.0007403563645424632, 0.02107305460400311, -0.249424483979713,
          0,           0,           0,           1;
  // Print the transformation 
  printf ("Method: using a Matrix4f\n");
  //std::cout << transform << std::endl;
  
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<PointT> ());
  pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
  plywriter.write<PointT> (outfile+"_transformed.ply", *transformed_cloud,false,false); 
}

bool LessSort (float a,float b) { return (a<b); }  
void precalZsign(pcl::PointCloud<PointT>::Ptr source,pcl::PointCloud<PointT>::Ptr target,
		 pcl::Correspondences& correspondences)
{
  double sign[4][2]={1,1,1,-1,-1,1,-1,-1};
  int base=55;
  int count_success=0;
  PointT s_base=source->points[base],t_base=target->points[base];   //base point
  pcl::Correspondences temp_correspondences=correspondences;
  for (int i= 0;  i< correspondences.size(); i ++)
  //for(pcl::Correspondences::iterator iter:correspondences)
  {
    if(i==base)continue;
    int index_query=correspondences[i].index_query;
    int index_match=correspondences[i].index_match;
   
    //calculate sign by min distance
    PointT p_s,p_t;
    double d_min=999,d_temp;
    vector<float> v_d;
    for(int j=0;j<4;j++)
    {
      PointT temp_s=source->points[index_query];
      PointT temp_t=target->points[index_match];
      temp_s.z=sign[j][0]*fabs(temp_s.z),temp_t.z=sign[j][1]*fabs(temp_t.z); 
      double temp_d1=pcl::euclideanDistance<PointT>(temp_s,s_base);
      double temp_d2=pcl::euclideanDistance<PointT>(temp_t,t_base);

      d_temp=fabs(temp_d1-temp_d2);
      v_d.push_back(d_temp);
      if(d_temp<d_min)
      {
	d_min=d_temp;p_s=temp_s;p_t=temp_t;
      }
    }
    std::sort(v_d.begin(),v_d.end(),LessSort);
    if(d_min>0.02) 
    {
      temp_correspondences.erase(temp_correspondences.begin()+i);continue;
    }
    else if(pcl::euclideanDistance<PointT>(p_s,source->points[index_query])==0
      &&pcl::euclideanDistance<PointT>(p_t,target->points[index_match])==0)
    {     
      count_success++;
      cout<<"precalZsign success "<<i<<endl; 
    }
    
    //uodate corresponces points;
    source->points[index_query]=p_s;
    target->points[index_match]=p_t;
  }
  correspondences=temp_correspondences;
  cout<<"count_success"<<count_success+1<<" in the correspondences "<<correspondences.size()<<endl;
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
  double R=0.315;    
  bag_dir.push_back("/media/whu/Research/04Research_PhD/01LRF_Calibation/data/dataset1_linuxdata20180408/T1-L1-0-L2-0.bag");
  bag_dir.push_back("/media/whu/Research/04Research_PhD/01LRF_Calibation/data/dataset1_linuxdata20180408/T2-L1-1-L2-1.bag");
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
  /*double R=0.325;    
  bag_dir.push_back("/media/whu/Research/04Research_PhD/01LRF_Calibation/data/dataset9_angle90_0_20180621/T1-L1-1-L2-1.bag");
  bag_dir.push_back("/media/whu/Research/04Research_PhD/01LRF_Calibation/data/dataset9_angle90_0_20180621/T2-L1-0-L2-0.bag");
  int flag[4]={1,1,-1,-1};
  double threshold[8][4]={
       0,0.7, 0,2,
       0,1, -2,0,    
       -1,0,-2,0,
       -0.4,0.4,0,2,
  };*/


  
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
	  if(coefficients_circle2d->values.size()==0 ) continue;
	  
	  temp.x=coefficients_circle2d->values[0];
	  temp.y=coefficients_circle2d->values[1];
	  temp.r=coefficients_circle2d->values[2];
	  temp.rms=coefficients_circle2d->values[3];
	  if(temp.r<R)	  temp.z=flag[i*2]*sqrt(R*R-temp.r*temp.r);
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
    mkdir(subdir.c_str(),0777);
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
	  if(temp.r<R/1.0)	  temp.z=flag[i*2]*sqrt(R*R-temp.r*temp.r);
	  else continue;
	  centers_v.push_back(temp);
	  
	  cout<<temp.stamp << ',' << temp.x<< ',' << temp.y << ','<<temp.z << ',' << temp.r<< ',' << temp.rms<<endl<<endl;
	  outFile << temp.stamp << ',' << temp.x<< ',' << temp.y << ',' << temp.z << ','<<temp.r<< ',' << temp.rms<< endl; 	  
	}
    }
    
    closedir(pdir);  
    outFile.close();       
  }
  
  // Fill in the cloud data  
  pcl::Correspondences all_correspondences ,good_correspondences ;
  pcl::PointCloud<PointT>::Ptr keypoints_v(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr keypoints_h(new pcl::PointCloud<PointT>); 
  keypoints_h->width    = centers_h.size();
  keypoints_h->height   = 1;
  keypoints_h->is_dense = false;
  keypoints_h->points.resize (keypoints_h->width * keypoints_h->height);
  for (vector<int>::size_type num_keypoints= 0; num_keypoints!= centers_h.size(); num_keypoints ++)
  {
    keypoints_h->points[num_keypoints].x=centers_h[num_keypoints].x;
    keypoints_h->points[num_keypoints].y=centers_h[num_keypoints].y;
    keypoints_h->points[num_keypoints].z=centers_h[num_keypoints].z;
    //keypoints_h->points[num_keypoints].label=centers_h[num_keypoints].rms;
  }
  keypoints_v->width    = centers_v.size();
  keypoints_v->height   = 1;
  keypoints_v->is_dense = false;
  keypoints_v->points.resize (keypoints_v->width * keypoints_v->height);
  for (vector<int>::size_type num_keypoints= 0; num_keypoints!= centers_v.size(); num_keypoints ++)
  {
    keypoints_v->points[num_keypoints].x=centers_v[num_keypoints].x;
    keypoints_v->points[num_keypoints].y=centers_v[num_keypoints].y;
    keypoints_v->points[num_keypoints].z=centers_v[num_keypoints].z;
    //keypoints_v->points[num_keypoints].label=centers_v[num_keypoints].rms;
  } 
  
  //4、时间戳对应
  // 写文件 
  subdir=workdir+"/cp_all";
  mkdir(subdir.c_str(),0777);
  outFile.open("cp_all.csv", ios::out);       
  outFile << "stamp1" << ',' << "x1" << ',' << "y1" << ',' << "z1" << ',' <<"r1" << ',' << "rms1" <<
  ',' << "stamp2" << ',' << "x2" << ',' << "y2" << ',' <<"z2" << ',' << "r2" << ',' << "rms2" <<endl;  
  for (vector<int>::size_type j= 0; j != centers_v.size(); j ++)
  {
    for (vector<int>::size_type i= 0; i != centers_h.size(); i ++)
    {
      long long temp=abs(centers_h[i].stamp-centers_v[j].stamp);
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
  
  //pre calculate Z sign in correspondences
  //precalZsign(keypoints_v,keypoints_h,all_correspondences);
  // 写文件 
  subdir=workdir+"/cp_r_R";
  mkdir(subdir.c_str(),0777);
  outFile.open("cp_r_R.csv", ios::out);   
  outFile << "stamp1" << ',' << "x1" << ',' << "y1" << ',' << "z1" << ',' <<"r1" << ',' << "rms1" <<
  ',' << "stamp2" << ',' << "x2" << ',' << "y2" << ',' <<"z2" << ',' << "r2" << ',' << "rms2" <<endl;  
  for (vector<int>::size_type j= 0; j != centers_v.size(); j ++)
  {
    for (vector<int>::size_type i= 0; i != centers_h.size(); i ++)
    {
      long long temp=abs(centers_h[i].stamp-centers_v[j].stamp);
      if(temp<25000/2&&centers_h[i].r<=0.707*R&&centers_v[i].r<=0.707*R)
      {	
	outFile << centers_h[i].stamp << ','  << centers_h[i].x<< ','<< centers_h[i].y << ',' << centers_h[i].z<< ',' 
	<< centers_h[i].r<< ',' << centers_h[i].rms
	<< ',' <<centers_v[j].stamp << ',' << centers_v[j].x<< ',' << centers_v[j].y << ',' << centers_v[j].z << ','
	<< centers_v[j].r<< ',' << centers_v[j].rms<< endl; 	
	pcl::Correspondence corr;
	corr.index_query = j;corr.index_match = i;corr.distance = 1;
	all_correspondences.push_back(corr);
      }	
    }
  }
  outFile.close();

  //pre calculate Z sign in correspondences
  //precalZsign(keypoints_v,keypoints_h,all_correspondences); 
  Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();
  cout<<"all_correspondences.size()="<<all_correspondences.size()<<endl;

  // Obtain the best transformation between the two sets of keypoints given the remaining correspondences
  TransformationEstimationSVD<PointT, PointT> trans_est;
  trans_est.estimateRigidTransformation (*keypoints_v, *keypoints_h, all_correspondences, pose);
  cout<<" TransformationEstimationSVD with all_correspondences: "<<endl;
  cout<<"pose = "<<pose<<endl;

  pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> rej;
  rej.setInputSource(keypoints_v);
  rej.setInputTarget(keypoints_h);
  rej.setMaximumIterations(10000);
  rej.setInlierThreshold(0.04);
  rej.getRemainingCorrespondences(all_correspondences,good_correspondences); 
  pose=rej.getBestTransformation();
  cout<<"good_correspondences.size()="<<good_correspondences.size()<<endl;
  cout<<" after rej.setInlierThreshold(0.01),rej.getBestTransformation() results: "<<endl;
  cout<<"pose = "<<pose<<endl;

  // 写文件 
  subdir=workdir+"/cp_rej";
  mkdir(subdir.c_str(),0777);
  outFile.open("cp_rej.csv", ios::out);    
  outFile << "stamp1" << ',' << "x1" << ',' << "y1" << ',' << "z1" << ',' <<"r1" << ',' << "rms1" <<
  ',' << "stamp2" << ',' << "x2" << ',' << "y2" << ',' <<"z2" << ',' << "r2" << ',' << "rms2" <<endl;    
  for(pcl::Correspondences::iterator it = good_correspondences.begin(); it != good_correspondences.end(); it ++)
  {
    int j=it->index_query,i=it->index_match;
    outFile << centers_h[i].stamp << ','  << centers_h[i].x<< ','<< centers_h[i].y << ',' << centers_h[i].z<< ',' 
    << centers_h[i].r<< ',' << centers_h[i].rms
    << ',' <<centers_v[j].stamp << ',' << centers_v[j].x<< ',' << centers_v[j].y << ',' << centers_v[j].z << ','
    << centers_v[j].r<< ',' << centers_v[j].rms<< endl;
  }
  outFile.close();

  //旋转矩阵转换为欧拉角
  Eigen::Matrix3d Rotation=Eigen::Matrix3d::Identity();
  Rotation  << pose(0,0),pose(1,1),pose(0,2),
	  pose(1,0),pose(1,1),pose(1,2),
	  pose(2,0),pose(2,1),pose(2,2);
  Eigen::Vector3d euler_angles=Rotation.eulerAngles(2,1,0)*180/PI;
  cout<<"Rotation = "<<Rotation<<endl;
  cout<<"intrinsic rotations YPR around rotating Z-Y-X"<<euler_angles.transpose()<<endl;
  cout<<"translations X-Y-Z "<<pose(0,3)<<" "<<pose(1,3)<< " "<<pose(2,3)<<endl; 
  
   //5.transform for validation
   //bag_dir[0]="/media/whu/Research/04Research_PhD/01LRF_Calibation/data/linuxdata20180408/validation.bag";
   workdir=bag_dir[1]+"_scan_ver/circle2d";
   //threshold[0]=-0.2,threshold[1]=0.5,threshold[2]=-1.5,threshold[3]=0;
   chdir(workdir.c_str());
   pdir = opendir(workdir.c_str()); 
   //清空子路径
   subdir=workdir+"/transform";
   mkdir(subdir.c_str(),0777);
  
    while((p = readdir(pdir)) != NULL)
    {
	//这里需要注意，linux平台下一个目录中有"."和".."隐藏文件，需要过滤掉
	//d_name是一个char数组，存放当前遍历到的文件名
	if(p->d_name[0] == 'v'&& strncmp(p->d_name + 18,".pcd_circle2d.ply", 5)==0)
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

