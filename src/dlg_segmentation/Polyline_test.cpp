#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJPolylineMesh.h所属头文件；

#include <PolylineMesh.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;
 
int 
main (int argc, char** argv)
{
    
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileOBJ("myobj.obj", mesh); 
  
  pcl::io::saveOBJFile("myobj2.obj", mesh);
  
  PolylineMesh linemesh;
  linemesh.cloud=mesh.cloud;

  for(size_t i=0;i<2;i++)
  {
    pcl::Vertices temp;
    temp.vertices.push_back(i+1),temp.vertices.push_back(i*2+2);  
    linemesh.polylines.push_back(temp);
  }
  for(size_t i=0;i<linemesh.polylines.size();i++)
  {
    pcl::Vertices temp =linemesh.polylines[i];

    std::cout<<temp<<std::endl;
  }
  
  pcl::io::saveOBJFile2("myobj3.obj", linemesh);

  return (0);
}
