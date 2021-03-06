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

#include <pcl/registration/gicp.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/nonfree/gpu.hpp>

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
					fusion2World();
					if (enableVis_)
					{

						
						//image_viewer_.showRGBImage(image_, width, height);
						cloud_viewer_.removeAllPointClouds();
						cloud_viewer_.addPointCloud(cloud_.makeShared());
						cloud_viewer_.spinOnce(10);
					}
					
					
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
		if (pre_cloud_.empty())
			return false;
//		boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());
//
		cv::Mat pre_img_mat(height, width, CV_8UC3, pre_image_);//not transform from rgb2bgr
		cv::Mat img_mat(height, width, CV_8UC3, image_);
//
//		//cv::cvtColor(pre_img_mat, pre_img_mat, CV_RGB2BGR);
//		//cv::cvtColor(img_mat, img_mat, CV_RGB2BGR);
//		
//		std::vector<cv::KeyPoint> pre_keypoints,keypoints;
//		/*cv::Ptr<cv::FeatureDetector> fdect = cv::FeatureDetector::create("FAST");
//		fdect->detect(pre_img_mat, pre_keypoints);
//		fdect->detect(img_mat, keypoints);*/
//
//		cv::SurfFeatureDetector detc(400);
//
//		detc.detect(img_mat, keypoints);
//		detc.detect(pre_img_mat, pre_keypoints);
//
//
//		cv::Mat pre_img_desc, img_desc;
//
//		/*cv::drawKeypoints(img_mat, keypoints, img_mat);
//		cv::drawKeypoints(pre_img_mat, pre_keypoints, pre_img_mat);
//		imshow("pre_img", pre_img_mat);
//		imshow("img",img_mat);
//		cvWaitKey(5);
//		*/
//		
//		cv::SurfDescriptorExtractor extractor;
//		extractor.compute(pre_img_mat, pre_keypoints, pre_img_desc);
//		extractor.compute(img_mat, keypoints, img_desc);
//
//		boost::posix_time::ptime t2(boost::posix_time::microsec_clock::local_time());
//		boost::posix_time::time_duration dt = t2 - t1;
//		std::cout << "extract a frame in: " << dt.total_milliseconds() / 1000.0 << std::endl;
//
//		std::vector<cv::DMatch> matches;
//
//		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
//		matcher->match(pre_img_desc, img_desc, matches);
//
//		boost::posix_time::ptime t3(boost::posix_time::microsec_clock::local_time());
//		dt = t3 - t2;
//		std::cout << "match a frame in: " << dt.total_milliseconds() / 1000.0 << std::endl;
//		
//		std::sort(matches.begin(), matches.end());
//		pcl::Correspondences corr;
//		pcl::PointCloud<pcl::PointXYZRGBA> pcl_keypoints, pcl_keypoints_pre;
//		int j = 0;
//		for (size_t i = 0; i < 32; i++)
//		{
//			int sid = matches[i].queryIdx;
//			int tid = matches[i].trainIdx;
//
//			int sind = int(pre_keypoints[sid].pt.y)*width + int(pre_keypoints[sid].pt.x);
//			int tind = int(keypoints[tid].pt.y)*width + int(keypoints[tid].pt.x);
//
//			
//			if (pcl_isfinite(cloud_.points[tind].x) && pcl_isfinite(pre_cloud_.points[sind].x))
//			{
//				pcl_keypoints.push_back(cloud_.points[tind]);
//				pcl_keypoints_pre.push_back(pre_cloud_.points[sind]);
//
//				pcl::Correspondence cori;
//				cori.index_query = j;
//				cori.index_match = j;
//				j++;
//				corr.push_back(cori);
//			}
//		}
//
//		Eigen::Matrix4f transform;
//
//		pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA, pcl::PointXYZRGBA> est;
//
//
//
//		est.estimateRigidTransformation(pcl_keypoints, pcl_keypoints_pre, corr, transform);
//
//		pcl::transformPointCloud(cloud_, cloud_, transform);
//		/*image_viewer_.showCorrespondences(pcl_keypoints_pre, pcl_keypoints, corr);
//		image_viewer_.spinOnce();
//*/
//		//world_ += cloud_;
//		/*pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> avg;
//		avg.setInputCloud(world_.makeShared());
//		avg.setLeafSize(0.05, 0.05, 0.05);
//		avg.filter(cloud_);*/
//		boost::posix_time::ptime t4(boost::posix_time::microsec_clock::local_time());
//		 dt = t4 - t3;
//		std::cout << "fusion a frame in: " << dt.total_milliseconds() / 1000.0 << std::endl; 
		boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());
		cv::gpu::GpuMat img_gpu_mat, img_gpu_mat_pre;

		cv::cvtColor(pre_img_mat, pre_img_mat, CV_RGB2GRAY);
		cv::cvtColor(img_mat, img_mat, CV_RGB2GRAY);
		img_gpu_mat.upload(img_mat);
		img_gpu_mat_pre.upload(pre_img_mat);

		//cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());

		cv::gpu::SURF_GPU surf;

		cv::gpu::GpuMat keypoints_gpu, keypoints_gpu_pre;
		cv::gpu::GpuMat img_desc_gpu, img_desc_gpu_pre;

		surf(img_gpu_mat, cv::gpu::GpuMat(), keypoints_gpu, img_desc_gpu);
		surf(img_gpu_mat_pre, cv::gpu::GpuMat(), keypoints_gpu_pre, img_desc_gpu_pre);

		cv::gpu::BFMatcher_GPU matcher_gpu(cv::NORM_L2);

		cv::gpu::GpuMat trainIdx, distance;
		matcher_gpu.matchSingle(img_desc_gpu_pre,img_desc_gpu, trainIdx, distance);

		std::vector<cv::KeyPoint> keypoints, keypoints_pre;
		std::vector<float>img_desc, img_desc_pre;

		std::vector<cv::DMatch> matches;

		surf.downloadKeypoints(keypoints_gpu, keypoints);
		surf.downloadKeypoints(keypoints_gpu_pre, keypoints_pre);

		surf.downloadDescriptors(img_desc_gpu, img_desc);
		surf.downloadDescriptors(img_desc_gpu_pre, img_desc_pre);

		cv::gpu::BFMatcher_GPU::matchDownload(trainIdx, distance, matches);

		boost::posix_time::ptime t2(boost::posix_time::microsec_clock::local_time());
		boost::posix_time::time_duration dt = t2 - t1;
		std::cout << "[1]match a frame in: " << dt.total_milliseconds() / 1000.0 << std::endl;

		std::sort(matches.begin(), matches.end());
				pcl::Correspondences corr;
				pcl::PointCloud<pcl::PointXYZRGBA> pcl_keypoints, pcl_keypoints_pre;
				int j = 0;
				for (size_t i = 0; i < 32; i++)
				{
					int sid = matches[i].queryIdx;
					int tid = matches[i].trainIdx;
		
					int sind = int(keypoints_pre[sid].pt.y)*width + int(keypoints_pre[sid].pt.x);
					int tind = int(keypoints[tid].pt.y)*width + int(keypoints[tid].pt.x);
		
					
					if (pcl_isfinite(cloud_.points[tind].x) && pcl_isfinite(pre_cloud_.points[sind].x))
					{
						pcl_keypoints.push_back(cloud_.points[tind]);
						pcl_keypoints_pre.push_back(pre_cloud_.points[sind]);
		
						pcl::Correspondence cori;
						cori.index_query = j;
						cori.index_match = j;
						j++;
						corr.push_back(cori);
					}
				}
		
				Eigen::Matrix4f transform;
		
				pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA, pcl::PointXYZRGBA> est;
		
		
				boost::posix_time::ptime t3(boost::posix_time::microsec_clock::local_time());
				dt = t3 - t2;
				std::cout << "[3]get correspondence in :" << dt.total_milliseconds() / 1000.0 << std::endl;

				est.estimateRigidTransformation(pcl_keypoints, pcl_keypoints_pre, corr, transform);

				boost::posix_time::ptime t4(boost::posix_time::microsec_clock::local_time());
				dt = t4 - t3;
				std::cout << "[4]estimate transfrom in: " << dt.total_milliseconds() / 1000.0 << std::endl;

				pcl::transformPointCloud(cloud_, cloud_, transform);

				boost::posix_time::ptime t5(boost::posix_time::microsec_clock::local_time());
				dt = t5 - t4;

				std::cout << "[5]transform cloud: " << dt.total_milliseconds() / 1000.0 << std::endl;
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