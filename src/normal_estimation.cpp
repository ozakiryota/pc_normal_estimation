#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>

class NormalEstimation{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscriber*/
		ros::Subscriber sub_;
		/*publisher*/
		ros::Publisher pub_;
		/*tool*/
		pcl::visualization::PCLVisualizer viewer {"pc_normal_estimation"};
		pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne_;
		/*buffer*/
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ {new pcl::PointCloud<pcl::PointXYZI>};
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr nc_ {new pcl::PointCloud<pcl::PointXYZINormal>};
		/*parameter*/
		double search_radius_;

	public:
		NormalEstimation();
		void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		void normalEstimation(void);
		void visualization(void);
		void publication(void);
};

NormalEstimation::NormalEstimation()
	: nh_private_("~")
{
	std::cout << "----- normal_estimation -----" << std::endl;
	/*parameter*/
	nh_private_.param("search_radius", search_radius_, 0.5);
	std::cout << "search_radius_ = " << search_radius_ << std::endl;
	/*subscriber*/
	sub_ = nh_.subscribe("/point_cloud", 1, &NormalEstimation::callback, this);
	/*publisher*/
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/normal_cloud", 1);
	/*setting*/
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
	ne_.setRadiusSearch(search_radius_);
}

void NormalEstimation::callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *pc_);
	copyPointCloud(*pc_, *nc_);
	normalEstimation();
	visualization();
	publication();
}

void NormalEstimation::normalEstimation(void)
{
	ne_.setInputCloud(pc_);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree_ (new pcl::search::KdTree<pcl::PointXYZI>());
	ne_.setSearchMethod(kdtree_);
	ne_.compute(*nc_);
}

void NormalEstimation::visualization(void)
{
	viewer.removeAllPointClouds();

	viewer.addPointCloud<pcl::PointXYZI>(pc_, "pc");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "pc");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc");

	viewer.addPointCloudNormals<pcl::PointXYZINormal>(nc_, 1, 0.5, "nc");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "nc");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "nc");

	viewer.spinOnce();
}

void NormalEstimation::publication(void)
{
	sensor_msgs::PointCloud2 ros_pc;
	pcl::toROSMsg(*nc_, ros_pc);
	pub_.publish(ros_pc);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pc_normal_estimation");

	NormalEstimation pc_normal_estimation;

	ros::spin();
}