// Prototype Kinect2 camera driver nodelet
// Author: Max Schwarz <max.schwarz@online.de>

#include "freenect2_camera.h"

#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include <pluginlib/class_list_macros.h>

extern "C"
{
extern void glfwInit();
}

namespace freenect2_camera
{

Freenect2Camera::Freenect2Camera()
 : m_device(0)
{
}

Freenect2Camera::~Freenect2Camera()
{
	if(m_device)
	{
		m_device->stop();
		m_device->close();
	}
}

void Freenect2Camera::onInit()
{
	ros::NodeHandle& nh = getPrivateNodeHandle();

	m_deviceName = ros::this_node::getName();

	glfwInit();

	m_device = m_freenect2.openDefaultDevice();

	if(!m_device)
	{
		throw std::runtime_error("Could not open device");
	}

	NODELET_INFO("Connected to Kinect2 device: %s, fw version: %s",
		m_device->getSerialNumber().c_str(),
		m_device->getFirmwareVersion().c_str()
	);

	m_it.reset(new image_transport::ImageTransport(nh));

	setupColor();
	setupDepth();

	m_device->setColorFrameListener(this);
	m_device->setIrAndDepthFrameListener(this);
	m_device->start();
}

void Freenect2Camera::setupColor()
{
	ros::NodeHandle& nh = getPrivateNodeHandle();

	// RGB setup
	std::string color_info_url;
	nh.param("rgb_info_url", color_info_url, std::string(""));

	std::stringstream ss;
	ss << m_device->getSerialNumber() << "_rgb";

	m_color_pub = m_it->advertiseCamera("rgb", 1);
	m_color_infoMgr.reset(
		new camera_info_manager::CameraInfoManager(
			getPrivateNodeHandle(), ss.str(), color_info_url
		)
	);

	sensor_msgs::CameraInfoPtr info;

	if(m_color_infoMgr->isCalibrated())
	{
		info = boost::make_shared<sensor_msgs::CameraInfo>(
			m_color_infoMgr->getCameraInfo()
		);
	}
	else
	{
		info.reset(new sensor_msgs::CameraInfo);

		libfreenect2::Freenect2Device::ColorCameraParams params = m_device->getColorCameraParams();

		/* We are reporting information about the *color* sensor here. */
		info->width = 1920;
		info->height = 1080;

		// No distortion
		info->D.resize(5, 0.0);
		info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

		// Simple camera matrix: square pixels (fx = fy), principal point at center
		info->K.assign(0.0);
		info->K[0] = params.fx;
		info->K[4] = params.fy;
		info->K[2] = params.cx;
		info->K[5] = params.cy;
		info->K[8] = 1.0;

		// No separate rectified image plane, so R = I
		info->R.assign(0.0);
		info->R[0] = info->R[4] = info->R[8] = 1.0;

		// Then P=K(I|0) = (K|0)
		info->P.assign(0.0);
		info->P[0] = params.fx;
		info->P[5] = params.fy;
		info->P[2] = info->K[2]; // cx
		info->P[6] = info->K[5]; // cy
		info->P[10] = 1.0;
	}

	info->header.frame_id = m_deviceName + "/rgb_optical";

	m_color_info = info;
}

void Freenect2Camera::setupDepth()
{
	ros::NodeHandle& nh = getPrivateNodeHandle();

	std::string depth_info_url;
	nh.param("depth_info_url", depth_info_url, std::string(""));

	std::stringstream ss;
	ss << m_device->getSerialNumber() << "_depth";

	m_depth_pub = m_it->advertiseCamera("depth", 1);
	m_depth_infoMgr.reset(
		new camera_info_manager::CameraInfoManager(
			getPrivateNodeHandle(), ss.str(), depth_info_url
		)
	);

	sensor_msgs::CameraInfoPtr info;

	if(m_depth_infoMgr->isCalibrated())
	{
		info = boost::make_shared<sensor_msgs::CameraInfo>(
			m_depth_infoMgr->getCameraInfo()
		);
	}
	else
	{
		info.reset(new sensor_msgs::CameraInfo);

		libfreenect2::Freenect2Device::IrCameraParams params = m_device->getIrCameraParams();

		/* We are reporting information about the *depth* sensor here. */
		info->width = 1920;
		info->height = 1080;

		// No distortion
		info->D.resize(5, 0.0);
		info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

		// Simple camera matrix: square pixels (fx = fy), principal point at center
		info->K.assign(0.0);
		info->K[0] = params.fx;
		info->K[4] = params.fy;
		info->K[2] = params.cx;
		info->K[5] = params.cy;
		info->K[8] = 1.0;

		// No separate rectified image plane, so R = I
		info->R.assign(0.0);
		info->R[0] = info->R[4] = info->R[8] = 1.0;

		// Then P=K(I|0) = (K|0)
		info->P.assign(0.0);
		info->P[0] = params.fx;
		info->P[5] = params.fy;
		info->P[2] = info->K[2]; // cx
		info->P[6] = info->K[5]; // cy
		info->P[10] = 1.0;
	}

	info->header.frame_id = m_deviceName + "/depth_optical";

	m_depth_info = info;
}

bool Freenect2Camera::onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame* frame)
{
	switch(type)
	{
		case libfreenect2::Frame::Color:
		{
			sensor_msgs::ImagePtr img(new sensor_msgs::Image);

			// FIXME: better stamp?
			img->header.stamp = ros::Time::now();
			img->header.frame_id = m_color_info->header.frame_id;

			img->encoding = sensor_msgs::image_encodings::BGR8;
			img->width = frame->width;
			img->height = frame->height;
			img->step = frame->bytes_per_pixel * img->width;

			// FIXME: Is this possible without copying?
			img->data.resize(img->step * img->height);
			memcpy(img->data.data(), frame->data, img->data.size());

			m_color_pub.publish(img, m_color_info);
			break;
		}
		case libfreenect2::Frame::Depth:
		{
			sensor_msgs::ImagePtr img(new sensor_msgs::Image);

			// FIXME: better stamp?
			img->header.stamp = ros::Time::now();
			img->header.frame_id = m_color_info->header.frame_id;

			img->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			img->width = frame->width;
			img->height = frame->height;
			img->step = frame->bytes_per_pixel * img->width;

			// FIXME: Is this possible without copying?
			img->data.resize(img->step * img->height);
			memcpy(img->data.data(), frame->data, img->data.size());

			m_depth_pub.publish(img, m_depth_info);
			break;
		}
	}

	delete frame;
	return true;
}

}

PLUGINLIB_EXPORT_CLASS(freenect2_camera::Freenect2Camera, nodelet::Nodelet)