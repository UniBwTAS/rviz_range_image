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

#ifndef RVIZ_RANGE_IMAGE_TEXTURE_H
#define RVIZ_RANGE_IMAGE_TEXTURE_H

#include <sensor_msgs/PointCloud2.h>

#include <OGRE/OgreImage.h>
#include <OGRE/OgreSharedPtr.h>
#include <OGRE/OgreTexture.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>

#include <rviz/default_plugin/point_cloud_transformer.h>

#include <stdexcept>

namespace rviz
{

class RangeImageTexture
{
  public:
    typedef V_PointCloudPoint CloudPoints;
    typedef std::shared_ptr<V_PointCloudPoint> CloudPointsPtr;
    typedef std::shared_ptr<const V_PointCloudPoint> CloudPointsConstPtr;

  public:
    RangeImageTexture();
    ~RangeImageTexture();

    void addPoints(uint32_t width, uint32_t height, const CloudPointsConstPtr& points);
    bool update();
    void clear();
    void flipped(bool flip);

    const Ogre::TexturePtr& getTexture()
    {
        return texture_;
    }

    uint32_t getWidth() const
    {
        return width_;
    }
    uint32_t getHeight() const
    {
        return height_;
    }

  private:
    CloudPointsConstPtr current_pc_;
    bool new_pc_;
    std::vector<uint8_t> data_;

    Ogre::TexturePtr texture_;
    Ogre::Image empty_image_;

    uint32_t input_width_;
    uint32_t input_height_;
    uint32_t width_;
    uint32_t height_;

    bool flipped_;
};

} // namespace rviz

#endif
