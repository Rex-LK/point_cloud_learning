#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>//基于法线找对应关系的头文件

using namespace std;
int
main(int argc, char** argv)
{
	// 加载源点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("cloud_data//P1.pcd", *source) == -1)
	{
		PCL_ERROR("读取目标点云失败 \n");
		return (-1);
	}
	cout << "从目标点云中读取 " << source->size() << " 个点" << endl;

	// 加载目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("cloud_data//P2.pcd", *target) == -1)
	{
		PCL_ERROR("读取源标点云失败 \n");
		return (-1);
	}
	cout << "从源点云中读取 " << target->size() << " 个点" << endl;

	//-------------------------法线估计--------------------------
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	
	n.setNumberOfThreads(8);  
	n.setInputCloud(source);
	n.setSearchMethod(tree);
	n.setKSearch(10);       
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	n.compute(*normals);   
	//初始化对象
	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointXYZ, pcl::PointXYZ, pcl::Normal>core;
	core.setInputSource(source);
	core.setSourceNormals(normals);
	core.setInputTarget(target);
	core.setKSearch(10);
	pcl::Correspondences all_correspondences;
	core.determineCorrespondences(all_correspondences, 10);//输入的是源点云上法线与目标点云上对应点之间的最大距离
	//core.determineReciprocalCorrespondences(all_correspondences);   //确定输入点云与目标点云之间的交互对应关系。

	//---------------------------可视化-------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	viewer->setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色
	// 对源点云着色可视化 (green).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>input_color(source, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(source, input_color, "input cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
	// 对目标点云着色可视化 (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(target, target_color, "target cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
	
	//----------------------对应关系可视化---------------------
	viewer->addCorrespondences<pcl::PointXYZ>(source, target, all_correspondences, "correspondence");
	viewer->initCameraParameters();
	viewer->addText("CorrespondenceEstimationNormalShooting", 10, 10, "text");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}

