#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
/* #include <thread> */
#include <omp.h>

class NormalEstimationMultiThread{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_pc;
		/*publish*/
		ros::Publisher pub_pc;
		/*pcl*/
		pcl::visualization::PCLVisualizer viewer {"Normal Estimation OMP"};
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
		/*sub class*/
		class NormalEstimation{
			private:
				pcl::PointCloud<pcl::PointNormal>::Ptr normals_ {new pcl::PointCloud<pcl::PointNormal>};
				size_t i_start;
				size_t i_end;
			public:
				NormalEstimation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, int start, int end, int skip);
				void Computation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, int start, int end, int skip);
				std::vector<int> KdtreeSearch(pcl::PointXYZ searchpoint, const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, double search_radius);
				pcl::PointCloud<pcl::PointNormal> GetNormals(void);
		};
	public:
		NormalEstimationMultiThread();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void GenerateThreads(void);
		void ClearPoints(void);
		void Visualization(void);
		void Publication(void);
};

NormalEstimationMultiThread::NormalEstimationMultiThread()
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &NormalEstimationMultiThread::CallbackPC, this);
	pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/normals", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.8, "axis");
	viewer.setCameraPosition(-30.0, 0.0, 10.0, 0.0, 0.0, 1.0);
}

void NormalEstimationMultiThread::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */

	pcl::fromROSMsg(*msg, *cloud);
	std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;
	std::cout << "cloud->is_dense = " << (bool)cloud->is_dense << std::endl;
	ClearPoints();
	/*generate kd-tree*/
	kdtree.setInputCloud(cloud);
	/*computation*/
	GenerateThreads();

	Publication();
	Visualization();
}

void NormalEstimationMultiThread::ClearPoints(void)
{
	normals->points.clear();
}

void NormalEstimationMultiThread::GenerateThreads(void)
{
	std::cout << "omp_get_max_threads() = " << omp_get_max_threads() << std::endl;

	const int num_threads = omp_get_max_threads();
	const int skip = 3;

	std::vector<NormalEstimation> multi_threads;

	double time_start = ros::Time::now().toSec();
	
	#pragma omp parallel for
	for(int i=0;i<num_threads;i++){
		/* std::cout << "omp_get_thread_num() = " << omp_get_thread_num() << std::endl; */
		int start = i*cloud->points.size()/num_threads;
		int end = (i+1)*cloud->points.size()/num_threads;
		NormalEstimation thread(cloud, kdtree, start, end, skip);
		// *normals += thread.GetNormals();
		multi_threads.push_back(thread);
	}
	for(int i=0;i<num_threads;i++)	*normals += multi_threads[i].GetNormals();

	std::cout << "time | total [s] = " << ros::Time::now().toSec() - time_start << std::endl;
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

NormalEstimationMultiThread::NormalEstimation::NormalEstimation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, int start, int end, int skip)
{
	Computation(cloud, kdtree, start, end, skip);
}

void NormalEstimationMultiThread::NormalEstimation::Computation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, int start, int end, int skip)
{
	double time_start = ros::Time::now().toSec();
	/* std::cout << "start index : " << start << std::endl;  */
	for(size_t i=start;i<end;i+=skip){
		/*search neighbor points*/
		std::vector<int> indices;
		const double search_radius = 0.3;
		indices = KdtreeSearch(cloud->points[i], kdtree, search_radius);
		/*compute normal*/
		float curvature;
		Eigen::Vector4f plane_parameters;
		pcl::computePointNormal(*cloud, indices, plane_parameters, curvature);
		/*create tmp object*/
		pcl::PointNormal tmp_normal;
		tmp_normal.x = cloud->points[i].x;
		tmp_normal.y = cloud->points[i].y;
		tmp_normal.z = cloud->points[i].z;
		tmp_normal.normal_x = plane_parameters[0];
		tmp_normal.normal_y = plane_parameters[1];
		tmp_normal.normal_z = plane_parameters[2];
		tmp_normal.curvature = curvature;
		flipNormalTowardsViewpoint(tmp_normal, 0.0, 0.0, 0.0, tmp_normal.normal_x, tmp_normal.normal_y, tmp_normal.normal_z);
		normals_->points.push_back(tmp_normal);
	}
	std::cout << "time | single thread [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}

std::vector<int> NormalEstimationMultiThread::NormalEstimation::KdtreeSearch(pcl::PointXYZ searchpoint, const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, double search_radius)
{
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	if(kdtree.radiusSearch(searchpoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
	return pointIdxRadiusSearch; 
}

pcl::PointCloud<pcl::PointNormal> NormalEstimationMultiThread::NormalEstimation::GetNormals(void)
{
	return *normals_;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "normal_estimation_omp");
	std::cout << "Normal Estimation OMP" << std::endl;
	
	NormalEstimationMultiThread normal_estimation_omp;


	ros::spin();
}
