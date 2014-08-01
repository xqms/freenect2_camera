// Prototype Kinect2 camera driver nodelet
// Author: Max Schwarz <max.schwarz@online.de>

#ifndef FREENECT2_CAMERA_H
#define FREENECT2_CAMERA_H

#include <nodelet/nodelet.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/threading.h>

#include <image_transport/camera_publisher.h>
#include <camera_info_manager/camera_info_manager.h>

namespace freenect2_camera
{

class Freenect2Camera : public nodelet::Nodelet, public libfreenect2::FrameListener
{
public:
	Freenect2Camera();
	virtual ~Freenect2Camera();

	virtual void onInit();
private:
	void setupColor();
	void setupDepth();

	virtual bool onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame* frame);

	boost::shared_ptr<image_transport::ImageTransport> m_color_it;
	sensor_msgs::CameraInfoConstPtr m_color_info;
	image_transport::CameraPublisher m_color_pub;
	boost::shared_ptr<camera_info_manager::CameraInfoManager> m_color_infoMgr;

	boost::shared_ptr<image_transport::ImageTransport> m_depth_it;
	sensor_msgs::CameraInfoConstPtr m_depth_info;
	image_transport::CameraPublisher m_depth_pub;
	boost::shared_ptr<camera_info_manager::CameraInfoManager> m_depth_infoMgr;

	sensor_msgs::CameraInfo m_ir_info;
	image_transport::CameraPublisher m_ir_pub;
	boost::shared_ptr<camera_info_manager::CameraInfoManager> m_ir_infoMgr;

	libfreenect2::Freenect2 m_freenect2;
	libfreenect2::Freenect2Device* m_device;

	std::string m_deviceName;

	bool m_initialized;
};

}

#endif
