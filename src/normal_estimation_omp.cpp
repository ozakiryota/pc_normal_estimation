#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl_conversions/pcl_conversions.h>

class PcNormalEstimationOmp{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscriber*/
		ros::Subscriber pc_sub_;
		/*publisher*/
		ros::Publisher pc_pub_;
		/*parameter*/
		double search_radius_;

	public:
		PcNormalEstimationOmp();
		void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		template<typename PcPtr, typename PointType, typename NcPtr, typename NormalType, typename KdTreePtr> void normalEstimation(PcPtr pc, PointType no_use_p, NcPtr nc, NormalType no_use_n, sensor_msgs::PointCloud2& pub_pc_msg, KdTreePtr kdtree);
		void publishMsg(const sensor_msgs::PointCloud2& pub_pc_msg);
};

PcNormalEstimationOmp::PcNormalEstimationOmp()
	: nh_private_("~")
{
	std::cout << "----- pc_normal_estimation_omp -----" << std::endl;
	/*parameter*/
	nh_private_.param("search_radius", search_radius_, 0.5);
	std::cout << "search_radius_ = " << search_radius_ << std::endl;
	/*subscriber*/
	pc_sub_ = nh_.subscribe("/point_cloud", 1, &PcNormalEstimationOmp::callback, this);
	/*publisher*/
	pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/normal_cloud", 1);
}

void PcNormalEstimationOmp::callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	sensor_msgs::PointCloud2 pub_pc_msg;

	std::string fields = pcl::getFieldsList(*msg);
	if(fields.find("intensity") != std::string::npos){
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr nc (new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::fromROSMsg(*msg, *pc);
		copyPointCloud(*pc, *nc);
		pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>());
		normalEstimation(pc, pc->points[0], nc, nc->points[0], pub_pc_msg, kdtree);
	}
	else{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointNormal>::Ptr nc (new pcl::PointCloud<pcl::PointNormal>);
		pcl::fromROSMsg(*msg, *pc);
		copyPointCloud(*pc, *nc);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>());
		normalEstimation(pc, pc->points[0], nc, nc->points[0], pub_pc_msg, kdtree);
	}
	publishMsg(pub_pc_msg);
}

template<typename PcPtr, typename PointType, typename NcPtr, typename NormalType, typename KdTreePtr>
void PcNormalEstimationOmp::normalEstimation(PcPtr pc, PointType no_use_p, NcPtr nc, NormalType no_use_n, sensor_msgs::PointCloud2& pub_pc_msg, KdTreePtr kdtree)
{
	pcl::NormalEstimationOMP<PointType, NormalType> neomp;
	neomp.setRadiusSearch(search_radius_);
	neomp.setInputCloud(pc);
	neomp.setSearchMethod(kdtree);
	neomp.compute(*nc);
	pcl::toROSMsg(*nc, pub_pc_msg);
}

void PcNormalEstimationOmp::publishMsg(const sensor_msgs::PointCloud2& pub_pc_msg)
{
	pc_pub_.publish(pub_pc_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pc_normal_estimation_omp");

	PcNormalEstimationOmp pc_normal_estimation_omp;

	ros::spin();
}