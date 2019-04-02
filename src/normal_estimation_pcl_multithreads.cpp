#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>

class NormalEstimationPCLMultiThreads{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_pc;
		/*publish*/
		ros::Publisher pub_pc;
		/*pcl*/
		pcl::visualization::PCLVisualizer viewer {"Normal Estimation PCL Multi Threads"};
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
		/*sub class*/
		class FittingWalls_{
			private:
				pcl::PointCloud<pcl::PointNormal>::Ptr normals_ {new pcl::PointCloud<pcl::PointNormal>};
			public:
				void Compute(NormalEstimationPCLMultiThreads &mainclass, size_t i_start, size_t i_end);
				void Merge(NormalEstimationPCLMultiThreads &mainclass);
		};
	public:
		NormalEstimationPCLMultiThreads();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void ClearPoints(void);
		std::vector<int> KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius);
		void Visualization(void);
		void Publication(void);
};

NormalEstimationPCLMultiThreads::NormalEstimationPCLMultiThreads()
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &NormalEstimationPCLMultiThreads::CallbackPC, this);
	pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/normals", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.8, "axis");
	viewer.setCameraPosition(-30.0, 0.0, 10.0, 0.0, 0.0, 1.0);
}

void NormalEstimationPCLMultiThreads::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "CALLBACK PC" << std::endl;
	pcl::fromROSMsg(*msg, *cloud);
	std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;
	ClearPoints();
	kdtree.setInputCloud(cloud);
	const int num_threads = std::thread::hardware_concurrency();
	std::cout << "number of threads: " << num_threads << std::endl;
	std::vector<std::thread> threads_fittingwalls;
	std::vector<FittingWalls_> objects;
	for(int i=0;i<num_threads;i++){
		FittingWalls_ tmp_object;
		objects.push_back(tmp_object);
	}
	double start_normal_est = ros::Time::now().toSec();
	for(int i=0;i<num_threads;i++){
		threads_fittingwalls.push_back(
			std::thread([i, num_threads, &objects, this]{
				objects[i].Compute(*this, i*cloud->points.size()/num_threads, (i+1)*cloud->points.size()/num_threads);
			})
		);
	}
	for(std::thread &th : threads_fittingwalls)	th.join();
	for(int i=0;i<num_threads;i++)	objects[i].Merge(*this);
	std::cout << "normal estimation time[s] = " << ros::Time::now().toSec() - start_normal_est << std::endl;

	Publication();
	Visualization();
}

void NormalEstimationPCLMultiThreads::ClearPoints(void)
{
	normals->points.clear();
}

void NormalEstimationPCLMultiThreads::FittingWalls_::Compute(NormalEstimationPCLMultiThreads &mainclass, size_t i_start, size_t i_end)
{
	const size_t skip_step = 3;
	for(size_t i=i_start;i<i_end;i+=skip_step){
		/*search neighbor points*/
		std::vector<int> indices;
		const double search_radius = 0.5;
		indices = mainclass.KdtreeSearch(mainclass.cloud->points[i], search_radius);
		/*compute normal*/
		float curvature;
		Eigen::Vector4f plane_parameters;
		pcl::computePointNormal(*mainclass.cloud, indices, plane_parameters, curvature);
		/*create tmp object*/
		pcl::PointNormal tmp_normal;
		tmp_normal.x = mainclass.cloud->points[i].x;
		tmp_normal.y = mainclass.cloud->points[i].y;
		tmp_normal.z = mainclass.cloud->points[i].z;
		tmp_normal.normal_x = plane_parameters[0];
		tmp_normal.normal_y = plane_parameters[1];
		tmp_normal.normal_z = plane_parameters[2];
		tmp_normal.curvature = curvature;
		flipNormalTowardsViewpoint(tmp_normal, 0.0, 0.0, 0.0, tmp_normal.normal_x, tmp_normal.normal_y, tmp_normal.normal_z);
		normals_->points.push_back(tmp_normal);
	}
}

void NormalEstimationPCLMultiThreads::FittingWalls_::Merge(NormalEstimationPCLMultiThreads &mainclass)
{
	*mainclass.normals += *normals_;
}

std::vector<int> NormalEstimationPCLMultiThreads::KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius)
{
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	if(kdtree.radiusSearch(searchpoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
	return pointIdxRadiusSearch; 
}

void NormalEstimationPCLMultiThreads::Visualization(void)
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

void NormalEstimationPCLMultiThreads::Publication(void)
{
	normals->header.stamp = cloud->header.stamp;
	normals->header.frame_id = cloud->header.frame_id;

	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(*normals, pc);
	pub_pc.publish(pc);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "normal_estimation_pcl_multithreads");
	std::cout << "Normal Estimation PCL Multi Threads" << std::endl;
	
	NormalEstimationPCLMultiThreads normal_estimation_pcl_multithreads;

	ros::spin();
}
