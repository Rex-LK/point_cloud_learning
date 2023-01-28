#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <pcl/registration/ia_kfpcs.h> //K4PCS算法头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

int main(int argc, char **argv)
{
    pcl::console::TicToc time;
    //----------------------------读取点云数据----------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/rex/Desktop/poind_cloud_learning/data/pcds/1.pcd", *target) == -1) // 加载目标点云
    {
        PCL_ERROR("读取目标点云失败 \n");
        return (-1);
    }
    cout << "从目标点云中读取 " << target->size() << " 个点" << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/rex/Desktop/poind_cloud_learning/data/pcds/2.pcd", *source) == -1) // 加载源点云
    {
        PCL_ERROR("读取源标点云失败 \n");
        return (-1);
    }
    cout << "从源点云中读取 " << source->size() << " 个点" << endl;
    time.tic();
    //--------------------------K4PCS算法进行配准------------------------------
    pcl::registration::KFPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> kfpcs;
    kfpcs.setInputSource(source);  // 源点云
    kfpcs.setInputTarget(target);  // 目标点云
    kfpcs.setApproxOverlap(0.7);   // 源和目标之间的近似重叠。
    kfpcs.setLambda(0.5);          // 平移矩阵的加权系数。(暂时不知道是干什么用的)
    kfpcs.setDelta(0.002, false);  // 配准后源点云和目标点云之间的距离
    kfpcs.setNumberOfThreads(6);   // OpenMP多线程加速的线程数
    kfpcs.setNumberOfSamples(200); // 配准时要使用的随机采样点数量
    // kfpcs.setMaxComputationTime(1000);//最大计算时间(以秒为单位)。
    pcl::PointCloud<pcl::PointXYZ>::Ptr kpcs(new pcl::PointCloud<pcl::PointXYZ>);
    kfpcs.align(*kpcs);

    cout << "KFPCS配准用时： " << time.toc() << " ms" << endl;
    cout << "变换矩阵：\n"
         << kfpcs.getFinalTransformation() << endl;
    // 使用创建的变换对为输入的源点云进行变换
    pcl::transformPointCloud(*source, *kpcs, kfpcs.getFinalTransformation());
    // 保存转换后的源点云作为最终的变换输出
    //  pcl::io::savePCDFileASCII ("transformed.pcd", *kpcs);
    //--------------------------可视化结果----------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("配准结果"));
    int v1 = 0;
    int v2 = 1;
    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->createViewPort(0.5, 0, 1, 1, v2);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->setBackgroundColor(0.05, 0, 0, v2);
    viewer->setWindowName("4PCS算法实现点云粗配准");
    viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
    viewer->addText("Registed point clouds", 10, 10, "v2_text", v2);
    // 原始点云绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 0, 255, 0);
    // 目标点云蓝色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 0, 0, 255);
    // 转换后的源点云红色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transe(kpcs, 255, 0, 0);
    // viewer->setBackgroundColor(255, 255, 255);
    viewer->addPointCloud(source, src_h, "source cloud", v1);
    viewer->addPointCloud(target, tgt_h, "target cloud", v1);
    viewer->addPointCloud(target, tgt_h, "target cloud1", v2);
    viewer->addPointCloud(kpcs, transe, "pcs cloud", v2);
    // 添加坐标系
    // viewer->addCoordinateSystem(0.1);
    // viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }

    return (0);
}
