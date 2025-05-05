// FILE: include/rviz_video_screen/video_screen_display.hpp

#pragma once
#include <rviz_common/visibility_control.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

#include <OgreMaterial.h>
#include <OgreManualObject.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreHardwarePixelBuffer.h>

namespace rviz_video_screen
{

class RVIZ_COMMON_PUBLIC VideoScreenDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  VideoScreenDisplay();
  ~VideoScreenDisplay() override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private:
  void subscribe();
  void unsubscribe();
  void processImage(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void createScreenQuad();
  void updateTexture();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rviz_common::properties::RosTopicProperty* topic_property_;

  Ogre::ManualObject* quad_ = nullptr;
  Ogre::SceneNode* node_ = nullptr;
  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;

  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  bool image_ready_ = false;
};

}  // namespace rviz_video_screen
