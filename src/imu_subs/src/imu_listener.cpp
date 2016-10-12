#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"



//sensor_msgs::CvBridge g_bridge;
boost::format g_format;
int g_count = 0;
std::string path_image;
std::string path_imu_measurement;

/**
 * This tutorial demonstrates simple receipt of IMU sensor data over the ROS system.
 */

/**
 * Callback function executes when new topic data comes.
 * Task of the callback function is to print data to screen.
 */
void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  //ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  //ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  //printf("!!\n");

	double imu_time = msg->header.stamp.sec + msg->header.stamp.nsec * pow(10,-9);
	FILE* fid_imu = fopen(path_imu_measurement.c_str(),"a");
	fprintf(fid_imu,"%10.9lf,%lf,%lf,%lf,%lf,%lf,%lf\n",imu_time,msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z,msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
	fclose(fid_imu);


}

void callback(const sensor_msgs::ImageConstPtr& msg_image, const sensor_msgs::CameraInfoConstPtr& info)
{

 	cv_bridge::CvImagePtr cv_ptr;
       try
       {
         cv_ptr = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::BGR8);
		if (&cv_ptr->image) {
			
			std_msgs::Header h = cv_ptr->header;


			char buf_img_filename[200];
			sprintf(buf_img_filename, "%s/%010d.%09d.jpg",path_image.c_str(),h.stamp.sec,h.stamp.nsec);

			cv::Mat img_resize;
			cv::resize(cv_ptr->image, img_resize, cv::Size(640,480));
			//std::string filename = (g_format % g_count % "jpg").str();
			std::string filename(buf_img_filename);
			imwrite(filename, img_resize);
			
		}else
		{
			ROS_WARN("Couldn't save image, no data!");
		}
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
       }
   
}

int main(int argc, char **argv)
{


	

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "imu_listener");

	long ns;
	time_t s;
	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);
	s = spec.tv_sec;
   	ns = spec.tv_nsec;

	long time_sec = s * pow(10,9) + ns;
	std::ostringstream time_sec_s;
	time_sec_s << time_sec;
	std::string forldername_dataset(time_sec_s.str());
	std::string path_img_data = "dataset/" + forldername_dataset;
	std::string path_imu_data = "dataset/" + forldername_dataset + "/imu" ;

	boost::filesystem::create_directories(path_img_data.c_str());
	boost::filesystem::create_directories(path_imu_data.c_str());

	path_image = path_img_data;
	path_imu_measurement = path_imu_data + "/data.csv";

	FILE* fid_imu = fopen(path_imu_measurement.c_str(),"w");
	fclose(fid_imu);



  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("imu/data", 1000, chatterCallback);


cv::namedWindow("test");


  ros::NodeHandle nh;
  g_format.parse("dataset/left%04i.%s");
  image_transport::ImageTransport it(nh);
  std::string topic = nh.resolveName("camera/image_raw");	
  image_transport::CameraSubscriber sub_cam = it.subscribeCamera(topic, 1, callback);
 




  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();




   



  return 0;
}
