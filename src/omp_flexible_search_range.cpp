#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <omp.h>

class NormalEstimationMultiThread{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc;
		/*publish*/
		ros::Publisher pub_pc;
		/*pcl*/
		pcl::visualization::PCLVisualizer viewer {"Normal Estimation Multi Thread"};
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
		/*parameters*/
		int skip;
		double search_radius_ratio;
	public:
		NormalEstimationMultiThread();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void Computation(void);
		std::vector<int> KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius);
		double Getdepth(pcl::PointXYZ point);
		void Visualization(void);
		void Publication(void);
};

NormalEstimationMultiThread::NormalEstimationMultiThread()
	:nhPrivate("~")
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &NormalEstimationMultiThread::CallbackPC, this);
	pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/normals", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.8, "axis");
	viewer.setCameraPosition(-30.0, 0.0, 10.0, 0.0, 0.0, 1.0);

	nhPrivate.param("skip", skip, 3);
	nhPrivate.param("search_radius_ratio", search_radius_ratio, 0.09);
	std::cout << "skip = " << skip << std::endl;
	std::cout << "search_radius_ratio = " << search_radius_ratio << std::endl;
}

void NormalEstimationMultiThread::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */

	pcl::fromROSMsg(*msg, *cloud);
	std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;
	normals->points.clear();
	normals->points.resize(cloud->points.size()/skip + 1);

	kdtree.setInputCloud(cloud);
	Computation();

	Publication();
	Visualization();
}

void NormalEstimationMultiThread::Computation(void)
{
	std::cout << "omp_get_max_threads() = " << omp_get_max_threads() << std::endl;

	double time_start = ros::Time::now().toSec();
	
	#pragma omp parallel for
	for(size_t i=0;i<cloud->points.size();i+=skip){
		/*search neighbor points*/
		double laser_distance = Getdepth(cloud->points[i]);
		double search_radius = search_radius_ratio*laser_distance;
		std::vector<int> indices = KdtreeSearch(cloud->points[i], search_radius);
		/*compute normal*/
		float curvature;
		Eigen::Vector4f plane_parameters;
		pcl::computePointNormal(*cloud, indices, plane_parameters, curvature);
		/*input*/
		int normal_index = i/skip;
		normals->points[normal_index].x = cloud->points[i].x;
		normals->points[normal_index].y = cloud->points[i].y;
		normals->points[normal_index].z = cloud->points[i].z;
		normals->points[normal_index].normal_x = plane_parameters[0];
		normals->points[normal_index].normal_y = plane_parameters[1];
		normals->points[normal_index].normal_z = plane_parameters[2];
		normals->points[normal_index].curvature = curvature;
		flipNormalTowardsViewpoint(cloud->points[i], 0.0, 0.0, 0.0, normals->points[normal_index].normal_x, normals->points[normal_index].normal_y, normals->points[normal_index].normal_z);
	}

	std::cout << "computation time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}

std::vector<int> NormalEstimationMultiThread::KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius)
{
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	if(kdtree.radiusSearch(searchpoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
	return pointIdxRadiusSearch; 
}

double NormalEstimationMultiThread::Getdepth(pcl::PointXYZ point)
{
	double depth = sqrt(
		point.x*point.x
		+ point.y*point.y
		+ point.z*point.z
	);
}

void NormalEstimationMultiThread::Visualization(void)
{
	viewer.removeAllPointClouds();

	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	
	viewer.addPointCloudNormals<pcl::PointNormal>(normals, 1, 0.5, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "normals");

	viewer.spinOnce();
}

void NormalEstimationMultiThread::Publication(void)
{
	normals->header.stamp = cloud->header.stamp;
	normals->header.frame_id = cloud->header.frame_id;

	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(*normals, pc);
	pub_pc.publish(pc);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "normal_estimation_multi_thread");
	std::cout << "Normal Estimation Multi Thread" << std::endl;
	
	NormalEstimationMultiThread normal_estimation_multi_thread;

	ros::spin();
}
