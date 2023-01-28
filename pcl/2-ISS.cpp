#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
// 点云内的关键点提取
int main(int, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("Armadillo.pcd", *cloud) == -1)
	{
		PCL_ERROR("Could not read file\n");
	}
	cout << "读取点云个数: " << cloud->points.size() << endl;

	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	//-------------------传递索引----------------------------
		//vector <int>(point_indices);
		//pcl::IndicesPtr indices(new vector <int>(point_indices));
	iss.setInputCloud(cloud);
	iss.setSearchMethod(tree);
	iss.setNumberOfThreads(4);     //初始化调度器并设置要使用的线程数
	iss.setSalientRadius(1.0f);  // 设置用于计算协方差矩阵的球邻域半径
	iss.setNonMaxRadius(1.5f);   // 设置非极大值抑制应用算法的半径
	iss.setThreshold21(0.65);     // 设定第二个和第一个特征值之比的上限
	iss.setThreshold32(0.5);     // 设定第三个和第二个特征值之比的上限
	iss.setMinNeighbors(10);       // 在应用非极大值抑制算法时，设置必须找到的最小邻居数
	
	iss.compute(*keypoints);

	/*for (size_t ii = 0; ii < keypoints->points.size();++ii) {

			point_indices.push_back(iss.getKeypointsIndices()->indices[ii]);

		};*/
	cout << "ISS_3D points 的提取结果为 " << keypoints->points.size() << endl;
	//pcl::io::savePCDFile("keypoints_iss_3d.pcd", *keypoints, true);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D ISS"));
	viewer->setBackgroundColor(255,255,255);
	viewer->setWindowName("ISS关键点提取");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0.0, 255, 0.0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer->addPointCloud<pcl::PointXYZ>(keypoints, "key cloud");//特征点
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "key cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "key cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}

	return 0;
}

