/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include <algorithm>
#include <iostream>
#include <map>

#include <OGRE/OgreTextureManager.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <boost/optional.hpp>

#include "range_image_texture.h"

namespace rviz
{
RangeImageTexture::RangeImageTexture()
{
    empty_image_.load("no_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    width_ = static_cast<int32_t>(empty_image_.getWidth());
    height_ = static_cast<int32_t>(empty_image_.getHeight());

    static uint32_t count = 0;
    std::stringstream ss;
    ss << "RangeImageTexture" << count++;
    texture_ = Ogre::TextureManager::getSingleton().loadImage(
        ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, empty_image_, Ogre::TEX_TYPE_2D, 0);
}

void RangeImageTexture::clear()
{
    texture_->unload();
    texture_->loadImage(empty_image_);
    width_ = static_cast<int32_t>(empty_image_.getWidth());
    height_ = static_cast<int32_t>(empty_image_.getHeight());
}

void RangeImageTexture::generateNewImage(const sensor_msgs::PointCloud2ConstPtr& msg,
                                         const CloudPoints& colorized_points)
{
    if (colorized_points.empty())
        return;

    width_ = msg->width;
    height_ = msg->height;
    data_.resize(width_ * height_ * 3);

    // check if we can mark NaN points
    boost::optional<sensor_msgs::PointCloud2ConstIterator<float>> iter_x;
    if (enable_separate_color_for_nan_points_)
    {
        try
        {
            iter_x = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "x");
        }
        catch (const std::runtime_error& e)
        {
            ROS_ERROR_STREAM_THROTTLE(3, "Unable to mark NaN points because points do not have 'x' field.");
        }
    }

    // iterate over cloud infos and insert pixels to range image
    uint32_t data_idx = 0;
    for (uint32_t i = 0; i < msg->width * msg->height; i++)
    {
        bool pos_is_nan = enable_separate_color_for_nan_points_ && iter_x && std::isnan(**iter_x);
        const Ogre::ColourValue c = pos_is_nan ? color_for_nan_points_ : colorized_points[i].color;
        data_[data_idx++] = static_cast<uint8_t>(c.b * 255);
        data_[data_idx++] = static_cast<uint8_t>(c.g * 255);
        data_[data_idx++] = static_cast<uint8_t>(c.r * 255);
        if (iter_x)
            ++(*iter_x);
    }

    // create ogre image
    Ogre::PixelFormat format = Ogre::PF_R8G8B8;
    Ogre::Image ogre_image;
    try
    {
        ogre_image.loadDynamicImage(data_.data(), width_, height_, 1, format);
    }
    catch (Ogre::Exception& e)
    {
        ROS_ERROR("Error loading image: %s", e.what());
        return;
    }
    texture_->unload();
    texture_->loadImage(ogre_image);
}

} // end of namespace rviz
