/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_IMAGE_DISPLAY_H
#define RVIZ_IMAGE_DISPLAY_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreSharedPtr.h>

#include <rviz/default_plugin/point_cloud_transformer.h>
#include <rviz/display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/render_panel.h>

#include <pluginlib/class_loader.hpp>

#include "range_image_texture.h"
#endif

namespace Ogre
{
class SceneNode;
class Rectangle2D;
} // namespace Ogre

namespace rviz
{

typedef boost::shared_ptr<PointCloudTransformer> PointCloudTransformerPtr;

struct TransformerInfo
{
    PointCloudTransformerPtr transformer;
    QList<Property*> color_props;

    std::string readable_name;
    std::string lookup_name;
};
typedef std::map<std::string, TransformerInfo> TransformerInfoMap;

/**
 * \class RangeImageDisplay
 *
 */
class RangeImageDisplay : public Display
{
    Q_OBJECT
  public:
    RangeImageDisplay();
    ~RangeImageDisplay() override;

    void update(float wall_dt, float ros_dt) override;

  protected Q_SLOTS:
    /** @brief Update topic and resubscribe */
    virtual void updateTopic();
    void causeRetransform();
    void updateColorTransformer();
    void setColorTransformerOptions(EnumProperty* prop);

  protected:
    void onInitialize() override;

    // overrides from Display
    void onEnable() override;
    void onDisable() override;

    /** @brief Reset display. */
    void reset() override;

    /** @brief ROS topic management. */
    virtual void subscribe();
    virtual void unsubscribe();

    /** @brief Incoming message callback.  Checks if the message pointer
     * is valid, increments messages_received_, then calls
     * processMessage(). */
    void incomingMessage(const sensor_msgs::PointCloud2::ConstPtr& msg);

    /** @brief Implement this to process the contents of a message.
     *
     * This is called by incomingMessage(). */
    virtual void processMessage(const sensor_msgs::PointCloud2::ConstPtr& msg);

  private:
    void loadTransformers();
    void updateTransformers(const sensor_msgs::PointCloud2ConstPtr& cloud);
    static void setPropertiesHidden(const QList<Property*>& props, bool hide);
    PointCloudTransformerPtr getColorTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void fillTransformerOptions(EnumProperty* prop, uint32_t mask);
    void colorizeCloudAndAddToTexture();

  protected:
    Ogre::SceneManager* img_scene_manager_{};
    RangeImageTexture texture_;
    RenderPanel* render_panel_{};

  private:
    pluginlib::ClassLoader<PointCloudTransformer>* transformer_class_loader_{nullptr};
    TransformerInfoMap transformers_;
    bool needs_retransform_{false};
    bool new_color_transformer_{false};

    ros::Subscriber sub_;
    sensor_msgs::PointCloud2ConstPtr last_msg_;

    Ogre::SceneNode* img_scene_node_{};
    Ogre::Rectangle2D* screen_rect_{};
    Ogre::MaterialPtr material_;

    uint32_t messages_received_{};

    RosTopicProperty* topic_property_;
    EnumProperty* color_transformer_property_;
    BoolProperty* flip_property_;
    BoolProperty* keep_aspect_ratio_property_;
};

} // namespace rviz

#endif
