// FILE: src/rviz_video_screen/video_screen_display.cpp

#include "rviz_video_screen/video_screen_display.hpp"

#include <cv_bridge/cv_bridge.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreTextureUnitState.h>
#include <OGRE/OgreImage.h>
#include <OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OgreTextureManager.h>
#include <OgreHardwarePixelBuffer.h>
#include <rviz_common/display_context.hpp>

namespace rviz_video_screen
{

VideoScreenDisplay::VideoScreenDisplay() {}

VideoScreenDisplay::~VideoScreenDisplay()
{
  unsubscribe();
  if (node_ && scene_manager_)
  {
    scene_manager_->destroySceneNode(node_);
  }
}

void VideoScreenDisplay::onInitialize()
{
  topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Image Topic",
    "/video/image",
    QString::fromStdString("sensor_msgs/msg/Image"),
    "The image topic to subscribe to.",
    this,
    nullptr);

  createScreenQuad();
}

void VideoScreenDisplay::createScreenQuad()
{
  texture_ = Ogre::TextureManager::getSingleton().createManual(
    "VideoTexture",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::TEX_TYPE_2D,
    640, 480,
    0,
    Ogre::PF_BYTE_BGR,
    Ogre::TU_DYNAMIC_WRITE_ONLY);

  material_ = Ogre::MaterialManager::getSingleton().create(
    "VideoMaterial",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  material_->getTechnique(0)->getPass(0)->createTextureUnitState("VideoTexture");
  material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);

  quad_ = scene_manager_->createManualObject();
  quad_->begin("VideoMaterial", Ogre::RenderOperation::OT_TRIANGLE_STRIP);
  float w = 1.0f, h = 0.75f;

  quad_->position(-w, -h, 0); quad_->textureCoord(0, 1);
  quad_->position(w, -h, 0);  quad_->textureCoord(1, 1);
  quad_->position(-w, h, 0);  quad_->textureCoord(0, 0);
  quad_->position(w, h, 0);   quad_->textureCoord(1, 0);
  quad_->end();

  node_ = scene_node_->createChildSceneNode();
  node_->attachObject(quad_);
}

void VideoScreenDisplay::onEnable()
{
  subscribe();
}

void VideoScreenDisplay::onDisable()
{
  unsubscribe();
}

void VideoScreenDisplay::subscribe()
{
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    topic_property_->getStdString(),
    rclcpp::SensorDataQoS(),
    std::bind(&VideoScreenDisplay::processImage, this, std::placeholders::_1));
}

void VideoScreenDisplay::unsubscribe()
{
  sub_.reset();
}

void VideoScreenDisplay::processImage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  latest_image_ = msg;
  image_ready_ = true;
}

void VideoScreenDisplay::update(float, float)
{
  if (image_ready_ && texture_.get()) {
    updateTexture();
    image_ready_ = false;
  }
}

void VideoScreenDisplay::updateTexture()
{
  try
  {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(latest_image_, "bgr8");

    Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture_->getBuffer();
    pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const Ogre::PixelBox& pixelBox = pixel_buffer->getCurrentLock();

    std::memcpy(pixelBox.data, cv_ptr->image.data, cv_ptr->image.total() * cv_ptr->image.elemSize());
    pixel_buffer->unlock();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
                 "Error updating texture: %s", e.what());
  }
}

}  // namespace rviz_video_screen

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_video_screen::VideoScreenDisplay, rviz_common::Display)
