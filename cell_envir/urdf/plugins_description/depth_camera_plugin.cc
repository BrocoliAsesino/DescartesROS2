#ifndef _DEPTH_CAMERA_PLUGIN_HH_
#define _DEPTH_CAMERA_PLUGIN_HH_

// #include <common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/transport/transport.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "interface/srv/camera_plugin_srv.hpp"

namespace gazebo
{
  class DepthCameraPlugin : public SensorPlugin, public rclcpp::Node
  {
    public: DepthCameraPlugin() : Node("depth_camera_plugin_node") {}

    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
      // pointer to a DepthCameraSensor pointer
      this->depthCameraSensor = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor);

      
      if (!this->depthCameraSensor)
      {
        gzerr << "Could not find a depth camera sensor\n";
        return;
      }
        // bind the DataCallBack method to the ConnectUpdated
        this->depthCameraSensor->ConnectUpdated(std::bind(&DepthCameraPlugin::DataCallBack, this));

        this->get_pcl_service = this->create_service<sensor_msgs::msg::PointCloud2>("get_point_cloud",
            std::bind(&DepthCameraPlugin::ServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
      
    }

    void DataCallBack(){
        
        
    }

    void ServiceCallback(const std::shared_ptr<interface::srv::CameraPluginSRV::Request> request,
                         std::shared_ptr<interface::srv::CameraPluginSRV::Response> response)
    {
      // TODO: fetch and fill in the point cloud data
      return response->point_cloud_msg_ 
    }

    private: 
        sensors::DepthCameraSensorPtr depthCameraSensor;
        rclcpp::Service<sensor_msgs::msg::PointCloud2>::SharedPtr get_pcl_service;
        sensor_msgs::msg::PointCloud2 point_cloud_msg_;
  };

  GZ_REGISTER_SENSOR_PLUGIN(DepthCameraPlugin)
}
#endif

