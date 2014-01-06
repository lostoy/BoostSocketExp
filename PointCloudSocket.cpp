#include "pcl_socket.cpp"

#include <boost/asio.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/openni_grabber.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/visualization/pcl_visualizer.h>


#include <iostream>
#include <string>

class PointCloudStreamer
{
public:
	PointCloudStreamer(pcl::Grabber &capture, bool enableServer = false, bool enableClient = false, bool enableVis = true) :capture_(capture), enableClient_(enableClient), enableServer_(enableServer), enableVis_(enableVis)
	{
		if (enableVis_)
			initVis();
		
	}

	void initVis()
	{
		viewer_.setBackgroundColor(0,0,0);
	}

	void grabRGBAframe(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		boost::mutex::scoped_try_lock lock(data_ready);
		if (!lock)
			return;

		pcl::copyPointCloud(*cloud, cloud_);
		data_ready_cond_.notify_one();
	}
	void mainLoop()
	{
		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&PointCloudStreamer::grabRGBAframe, this, _1);
		capture_.registerCallback(f);
		capture_.start();
		boost::unique_lock<boost::mutex> lock(data_ready);
		if (enableVis_)
			while (!viewer_.wasStopped())
			{
				bool has_data = data_ready_cond_.timed_wait(lock, boost::posix_time::millisec(100));
				
				if (has_data)
				{
					viewer_.removeAllPointClouds();
					viewer_.addPointCloud(cloud_.makeShared());
				}
			
				viewer_.spinOnce();
			}
	}
private:
	boost::asio::io_service io_service_;
	SocketServer::SocketServerPtr server_;
	SocketClient::SocketClientPtr client_;
	pcl::Grabber& capture_;
	pcl::visualization::PCLVisualizer viewer_;
	pcl::PointCloud<pcl::PointXYZRGBA> cloud_;

	bool enableServer_;
	bool enableClient_;
	bool enableVis_;
	boost::mutex data_ready;
	boost::condition_variable data_ready_cond_;
};
int main()
{
	boost::shared_ptr<pcl::Grabber> capture;
	try
	{

		capture.reset(new pcl::OpenNIGrabber());
	}
	catch (const pcl::PCLException&){ std::cout << "wrong open device!" << std::endl; exit(-1); }

	PointCloudStreamer app(*capture);
	app.mainLoop();
}