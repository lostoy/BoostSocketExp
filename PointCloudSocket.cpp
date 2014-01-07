#include "SocketServer.hpp"


#include <boost/asio.hpp>
#include <boost/timer.hpp>
#include <boost/chrono.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/openni_grabber.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <string>



class PointCloudStreamer
{
public:
	PointCloudStreamer(pcl::Grabber &capture, bool enableServer = false, bool enableClient = false, bool enableVis = true) :capture_(capture), enableClient_(enableClient), enableServer_(enableServer), enableVis_(enableVis)
	{
		exit_ = false;
		if (enableVis_)
			initVis();
		if (enableServer_)
			initServer();
		
	}
	~PointCloudStreamer()
	{
		
		if (enableServer_)
		{
			io_service_.stop();
			socketThread_->join();

		}

	}
	void initVis()
	{
		cloud_viewer_.setBackgroundColor(0,0,0);
		image_viewer_.registerKeyboardCallback(boost::bind(&PointCloudStreamer::keyboard_callback, this, _1));
	}

	void initServer()
	{
		
		server_ =boost::shared_ptr<SocketServer>(new SocketServer(io_service_));
		server_->start();
		socketThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&boost::asio::io_service::run,&io_service_)));
		
	}
	void grabRGBAframe(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		boost::mutex::scoped_try_lock lock(data_ready);
		if (!lock||exit_)
			return;
		boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());
		if (!cloud_.empty())
		{
			pcl::copyPointCloud(cloud_, pre_cloud_);
			memcpy(pre_image_, image_,width*height*3);
		}
		pcl::copyPointCloud(*cloud, cloud_);
		size_t j = 0;
		for (size_t i = 0; i < cloud->size(); i++)
		{

			image_[j++] = cloud->points[i].r;
			image_[j++] = cloud->points[i].g;
			image_[j++] = cloud->points[i].b;
		}
		boost::posix_time::ptime t2(boost::posix_time::microsec_clock::local_time());
		boost::posix_time::time_duration dt = t2 - t1;
		std::cout << "grab a frame in: "<<dt.total_milliseconds()/1000.0 << std::endl;

		data_ready_cond_.notify_one();
	}

	//void grabImageframe(const boost::shared_ptr<openni_wrapper::Image> & image)
	//{
	//	boost::mutex::scoped_try_lock lock(data_ready);
	//	if (!lock)
	//		return;
	//	image->fillRGB(width, height, image_);
	//	data_ready_cond_.notify_one();
	//}
	void mainLoop()
	{
		using namespace openni_wrapper;
		typedef boost::shared_ptr<Image> ImagePtr;
		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f_cloud = boost::bind(&PointCloudStreamer::grabRGBAframe, this, _1);
	/*	boost::function<void(const ImagePtr&)>  f_image = boost::bind(&PointCloudStreamer::grabImageframe, this, _1);*/
		capture_.registerCallback(f_cloud);
		/*capture_.registerCallback(f_image);*/
		capture_.start();
		boost::unique_lock<boost::mutex> lock(data_ready);
		
			while (!exit_&&!cloud_viewer_.wasStopped()&&!image_viewer_.wasStopped())
			{
				bool has_data = data_ready_cond_.timed_wait(lock, boost::posix_time::millisec(60));
				
				if (has_data)
				{
					if (enableVis_)
					{

		//				cloud_viewer_.removeAllPointClouds();
						/*viewer_.addPointCloud(cloud_.makeShared());*/
						//image_viewer_.showRGBImage(image_, width, height);
						//cloud_viewer_.spinOnce(3);
					}
					fusion2World();
					if (enableServer_)
					{
						DispatchData();
					}
				}
			
				
			}
			capture_.stop();

	}
private:

	size_t DispatchData()
	{
		for (std::vector<SocketStreamer::SocketStreamerPtr>::iterator it = server_->socketList_.begin(); it != server_->socketList_.end(); it++)
		{

			//if (server_->socketList_[i])
			SocketStreamer & itref = *(*it);
			itref.sendData();
		}
		return server_->socketList_.size();

	}

	bool fusion2World()
	{
		cv::Mat pre_img_mat(height, width, CV_8UC3, pre_image_);//not transform from rgb2bgr
		cv::Mat img_mat(height, width, CV_8UC3, image_);

		//cv::cvtColor(pre_img_mat, pre_img_mat, CV_RGB2BGR);
		//cv::cvtColor(img_mat, img_mat, CV_RGB2BGR);
		
		std::vector<cv::KeyPoint> pre_keypoints,keypoints;
		cv::Ptr<cv::FeatureDetector> fdect = cv::FeatureDetector::create("FAST");
		fdect->detect(pre_img_mat, pre_keypoints);
		fdect->detect(img_mat, keypoints);

		cv::Mat pre_img_desc, img_desc;
		cv::drawKeypoints(img_mat, keypoints, img_mat);
		cv::drawKeypoints(pre_img_mat, pre_keypoints, pre_img_mat);
		imshow("pre_img", pre_img_mat);
		imshow("img",img_mat);
		cvWaitKey(60);
		//cv::SurfDescriptorExtractor extractor;
		//extractor.compute(pre_img_mat, pre_keypoints, pre_img_desc);
		//extractor.compute(img_mat, keypoints, img_desc);

		//std::vector<cv::DMatch> matches;

		//cv::FlannBasedMatcher matcher;


		//matcher.match(pre_img_desc, img_desc, matches);
		//std::sort(matches.begin(), matches.end());
		

		
		return true;
	}
	
	void keyboard_callback(const pcl::visualization::KeyboardEvent &e)
	{
		if (e.keyUp())
		{
			int key = e.getKeyCode();
			if (key == (int)'q')
				exit_ = true;
		
		}
	}
	static const int width = 640, height = 480;
	boost::asio::io_service io_service_;
	SocketServerPtr server_;
	SocketClient::SocketClientPtr client_;
	pcl::Grabber& capture_;

	pcl::visualization::PCLVisualizer cloud_viewer_;
	pcl::visualization::ImageViewer image_viewer_;

	pcl::PointCloud<pcl::PointXYZRGBA> cloud_;
	pcl::PointCloud<pcl::PointXYZRGBA> pre_cloud_;
	pcl::PointCloud<pcl::PointXYZRGBA> world_;
	unsigned char image_[width*height * 3];
	unsigned char pre_image_[width*height * 3];

	bool enableServer_;
	bool enableClient_;
	bool enableVis_;
	bool exit_;

	boost::mutex data_ready;
	boost::condition_variable data_ready_cond_;

	boost::shared_ptr<boost::thread> socketThread_;

	
};

int main()
{
	boost::shared_ptr<pcl::Grabber> capture;
	try
	{

		capture.reset(new pcl::OpenNIGrabber());
	}
	catch (const pcl::PCLException&){ std::cout << "wrong open device!" << std::endl; exit(-1); }
	boost::shared_ptr<PointCloudStreamer> app =boost::shared_ptr<PointCloudStreamer>( new PointCloudStreamer(*capture));

	
	app->mainLoop();
	return 0;
}