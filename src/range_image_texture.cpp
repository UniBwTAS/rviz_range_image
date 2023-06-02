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

#include "range_image_texture.h"

namespace rviz
{
RangeImageTexture::RangeImageTexture() : new_pc_(false)
{
    empty_image_.load("no_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    width_ = empty_image_.getWidth();
    height_ = empty_image_.getHeight();

    static uint32_t count = 0;
    std::stringstream ss;
    ss << "RangeImageTexture" << count++;
    texture_ = Ogre::TextureManager::getSingleton().loadImage(
        ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, empty_image_, Ogre::TEX_TYPE_2D, 0);
}

RangeImageTexture::~RangeImageTexture()
{
    current_pc_.reset();
}

void RangeImageTexture::clear()
{
    texture_->unload();
    texture_->loadImage(empty_image_);
    width_ = empty_image_.getWidth();
    height_ = empty_image_.getHeight();

    new_pc_ = false;
    current_pc_.reset();
}

bool RangeImageTexture::update()
{
    if (!current_pc_ || !new_pc_)
    {
        return false;
    }

    new_pc_ = false;

    if (current_pc_->empty() || width_ == 0 || height_ == 0)
    {
        return false;
    }

    data_.resize(input_width_ * input_height_ * 3);
    uint32_t data_idx = 0;
    if(!flipped_)
    {
        width_ = input_width_;
        height_ = input_height_;
        for (const auto& point : *current_pc_)
        {
            data_[data_idx++] = static_cast<uint8_t>(point.color.b * 255);
            data_[data_idx++] = static_cast<uint8_t>(point.color.g * 255);
            data_[data_idx++] = static_cast<uint8_t>(point.color.r * 255);
        }
    } else {
        width_ = input_height_;
        height_ = input_width_;
        for (uint32_t row_idx = 0; row_idx < height_; row_idx++)
        {
            for (uint32_t column_idx = 0; column_idx < width_; column_idx++)
            {
                data_idx = (row_idx * width_ + column_idx) * 3;
                const PointCloud::Point& point = (*current_pc_)[column_idx * height_ + (height_ - 1 - row_idx)];
                data_[data_idx++] = static_cast<uint8_t>(point.color.b * 255);
                data_[data_idx++] = static_cast<uint8_t>(point.color.g * 255);
                data_[data_idx++] = static_cast<uint8_t>(point.color.r * 255);
            }
        }
    }

    Ogre::PixelFormat format = Ogre::PF_R8G8B8;
    Ogre::Image ogre_image;

    try
    {
        ogre_image.loadDynamicImage(data_.data(), width_, height_, 1, format);
    }
    catch (Ogre::Exception& e)
    {
        ROS_ERROR("Error loading image: %s", e.what());
        return false;
    }

    texture_->unload();
    texture_->loadImage(ogre_image);

    return true;
}

void RangeImageTexture::addPoints(uint32_t width, uint32_t height, const CloudPointsConstPtr& points)
{
    current_pc_ = points;
    input_width_ = width;
    input_height_ = height;
    new_pc_ = true;
}

void RangeImageTexture::flipped(bool flip)
{
    flipped_ = flip;
    new_pc_ = true;
}

} // end of namespace rviz
