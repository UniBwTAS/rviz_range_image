#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreTextureManager.h>
#include <boost/bind.hpp>

#include <rviz/default_plugin/point_cloud_transformer.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/compatibility.h>
#include <rviz/render_panel.h>
#include <rviz/validate_floats.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "range_image_display.h"

namespace rviz
{
RangeImageDisplay::RangeImageDisplay() : Display(), texture_()
{
    topic_property_ =
        new RosTopicProperty("Topic",
                             "",
                             QString::fromStdString(ros::message_traits::datatype<sensor_msgs::PointCloud2>()),
                             "sensor_msgs::PointCloud2 topic to subscribe to.",
                             this,
                             SLOT(updateTopic()));

    color_transformer_property_ = new EnumProperty("Color Transformer",
                                                   "",
                                                   "Set the transformer to use to set the color of the points.",
                                                   this,
                                                   SLOT(updateColorTransformer()),
                                                   this);
    connect(color_transformer_property_,
            SIGNAL(requestOptions(EnumProperty*)),
            this,
            SLOT(setColorTransformerOptions(EnumProperty*)));

    flip_property_ = new BoolProperty("Flip", false, "Flips width and height.", this, SLOT(resetForFlip()), this);

    upside_down_property_ =
        new BoolProperty("Upside Down", false, "Show range image upside down.", this, SLOT(resetForFlip()), this);

    keep_aspect_ratio_property_ = new BoolProperty("Keep Aspect Ratio",
                                                   true,
                                                   "Keep the aspect ratio of range image while resizing window.",
                                                   this,
                                                   SLOT(causeRetransform()),
                                                   this);

    mark_nan_pixels_property_ = new BoolProperty("Mark NaN Pixels",
                                                 false,
                                                 "Whether to mark NaN pixels with a specific color.",
                                                 this,
                                                 SLOT(causeRetransform()),
                                                 this);

    nan_color_property_ = new ColorProperty("NaN Pixel Color",
                                            Qt::red,
                                            "The color of NaN pixels.",
                                            mark_nan_pixels_property_,
                                            SLOT(causeRetransform()),
                                            this);

    enable_streaming_property_ =
        new BoolProperty("Enable streaming",
                         false,
                         "Whether multiple sub-range images (messages) should be accumulated horizontally to display a "
                         "range image. Please choose a criterion for removing old sub-range images.",
                         this,
                         SLOT(causeRetransform()),
                         this);

    streaming_max_columns_property_ =
        new IntProperty("Maximum Columns",
                        0,
                        "Maximum accumulated number of column of all messages. Zero means disabled.",
                        enable_streaming_property_,
                        SLOT(causeRetransform()),
                        this);
}

RangeImageDisplay::~RangeImageDisplay()
{
    unsubscribe();

    if (initialized())
    {
        delete render_panel_;
        delete screen_rect_;
        removeAndDestroyChildNode(img_scene_node_->getParentSceneNode(), img_scene_node_);
    }
}

void RangeImageDisplay::onInitialize()
{
    transformer_class_loader_ =
        new pluginlib::ClassLoader<PointCloudTransformer>("rviz", "rviz::PointCloudTransformer");
    loadTransformers();

    {
        static uint32_t count = 0;
        std::stringstream ss;
        ss << "RangeImageDisplay" << count++;
        img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, ss.str());
    }

    img_scene_node_ = img_scene_manager_->getRootSceneNode()->createChildSceneNode();

    {
        static int count = 0;
        std::stringstream ss;
        ss << "RangeImageDisplayObject" << count++;

        screen_rect_ = new Ogre::Rectangle2D(true);
        screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
        screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

        ss << "Material";
        material_ = Ogre::MaterialManager::getSingleton().create(
            ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material_->setSceneBlending(Ogre::SBT_REPLACE);
        material_->setDepthWriteEnabled(false);
        material_->setReceiveShadows(false);
        material_->setDepthCheckEnabled(false);

        material_->getTechnique(0)->setLightingEnabled(false);
        Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
        tu->setTextureName(texture_.getTexture()->getName());
        tu->setTextureFiltering(Ogre::TFO_NONE);

        material_->setCullingMode(Ogre::CULL_NONE);
        Ogre::AxisAlignedBox aabInf;
        aabInf.setInfinite();
        screen_rect_->setBoundingBox(aabInf);
        setMaterial(*screen_rect_, material_);
        img_scene_node_->attachObject(screen_rect_);
    }

    render_panel_ = new RenderPanel();
    render_panel_->getRenderWindow()->setAutoUpdated(false);
    render_panel_->getRenderWindow()->setActive(false);

    render_panel_->resize(640, 480);
    render_panel_->initialize(img_scene_manager_, context_);

    setAssociatedWidget(render_panel_);

    render_panel_->setAutoRender(false);
    render_panel_->setOverlaysEnabled(false);
    render_panel_->getCamera()->setNearClipDistance(0.01f);
}

void RangeImageDisplay::incomingMessage(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!msg || context_->getFrameManager()->getPause())
    {
        return;
    }

    ++messages_received_;
    setStatus(StatusProperty::Ok, "Point Cloud", QString::number(messages_received_) + " point clouds received");

    emitTimeSignal(msg->header.stamp);

    processMessage(msg);
}

void RangeImageDisplay::reset()
{
    Display::reset();

    messages_received_ = 0;
    setStatus(StatusProperty::Warn, "Point Cloud", "No point cloud received");

    texture_.clear();
    existing_columns_.clear();
    render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
}

void RangeImageDisplay::subscribe()
{
    if (!isEnabled())
    {
        return;
    }

    try
    {
        sub_.shutdown();

        if (!topic_property_->getTopicStd().empty())
        {
            sub_ =
                update_nh_.subscribe(topic_property_->getTopicStd(), 10000, &RangeImageDisplay::incomingMessage, this);
        }
        setStatus(StatusProperty::Ok, "Topic", "OK");
    }
    catch (ros::Exception& e)
    {
        setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }
}

void RangeImageDisplay::unsubscribe()
{
    sub_.shutdown();
}

void RangeImageDisplay::updateTopic()
{
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
}

void RangeImageDisplay::onEnable()
{
    subscribe();
    render_panel_->getRenderWindow()->setActive(true);
}

void RangeImageDisplay::onDisable()
{
    render_panel_->getRenderWindow()->setActive(false);
    unsubscribe();
    reset();
}

void RangeImageDisplay::update(float wall_dt, float ros_dt)
{
    Q_UNUSED(wall_dt)
    Q_UNUSED(ros_dt)

    // remove old columns
    if (enable_streaming_property_->getBool())
    {
        auto it = existing_columns_.begin();
        while (existing_columns_.size() > streaming_max_columns_property_->getInt())
            it = existing_columns_.erase(it);
    }

    transformClouds();

    // make sure the aspect ratio of the image is preserved
    auto win_width = static_cast<float>(render_panel_->width());
    auto win_height = static_cast<float>(render_panel_->height());

    auto img_width = static_cast<float>(texture_.getWidth());
    auto img_height = static_cast<float>(texture_.getHeight());

    if (img_width != 0 && img_height != 0 && win_width != 0 && win_height != 0)
    {
        if (keep_aspect_ratio_property_->getBool())
        {
            float img_aspect = img_width / img_height;
            float win_aspect = win_width / win_height;

            if (img_aspect > win_aspect)
            {
                screen_rect_->setCorners(
                    -1.0f, 1.0f * win_aspect / img_aspect, 1.0f, -1.0f * win_aspect / img_aspect, false);
            }
            else
            {
                screen_rect_->setCorners(
                    -1.0f * img_aspect / win_aspect, 1.0f, 1.0f * img_aspect / win_aspect, -1.0f, false);
            }
        }
        else
        {
            screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f, false);
        }
    }

    render_panel_->getRenderWindow()->update();

    // update visibility of transformer properties
    if (new_color_transformer_)
    {
        auto it = transformers_.begin();
        auto end = transformers_.end();
        for (; it != end; ++it)
        {
            const std::string& name = it->first;
            TransformerInfo& info = it->second;

            setPropertiesHidden(info.color_props, name != color_transformer_property_->getStdString());
        }
    }

    new_color_transformer_ = false;
}

void RangeImageDisplay::processMessage(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!enable_streaming_property_->getBool())
        existing_columns_.clear();

    // check if point cloud is valid otherwise ignore it
    if (msg->width == 0 || msg->height == 0 || msg->data.size() != msg->width * msg->height * msg->point_step)
    {
        ROS_ERROR("Error in Range Image Visualization: Invalid Message");
        return;
    }

    // do not update color transformers too frequently
    if (existing_columns_.empty())
        updateTransformers(msg);

    // split this message into multiple column msgs in order to keep the pipeline afterwards simple
    // get variables required for rotations
    bool not_flipped = !flip_property_->getBool();
    bool not_upside_down = !upside_down_property_->getBool();
    uint32_t width, height, stride_in_src;
    if (not_flipped)
    {
        width = msg->width;
        height = msg->height;
        stride_in_src = msg->row_step;
    }
    else
    {
        width = msg->height;
        height = msg->width;
        stride_in_src = msg->point_step;
    }
    auto stride_in_dst = static_cast<int32_t>(not_upside_down ? msg->point_step : -msg->point_step);
    // check if height of new columns fits to height of existing range image
    if (!existing_columns_.empty() && existing_columns_.back()->height != height)
    {
        ROS_ERROR("Error in Range Image Visualization: Height of new message does not match existing range image. "
                  "Maybe you have to uncheck 'Streaming' or toggle 'Flip' option.");
        return;
    }
    for (int column_index = 0; column_index < width; column_index++)
    {
        sensor_msgs::PointCloud2::Ptr column_msg(new sensor_msgs::PointCloud2());
        column_msg->header = msg->header; // TODO: stamp?
        column_msg->height = height;
        column_msg->width = 1;
        column_msg->fields = msg->fields;
        column_msg->is_bigendian = msg->is_bigendian;
        column_msg->point_step = msg->point_step;
        column_msg->row_step = msg->point_step;
        column_msg->is_dense = msg->is_dense;

        column_msg->data.resize(height * msg->point_step);
        uint32_t byte_position_src;
        byte_position_src =
            not_flipped ? column_index* msg->point_step : byte_position_src = column_index * height * msg->point_step;
        uint32_t byte_position_dst = not_upside_down ? 0 : msg->point_step * (height - 1);
        for (int row_index = 0; row_index < height; row_index++)
        {
            memcpy(column_msg->data.data() + byte_position_dst, msg->data.data() + byte_position_src, msg->point_step);
            byte_position_src += stride_in_src;
            byte_position_dst += stride_in_dst;
        }
        existing_columns_.push_back(column_msg);
        new_message_available = true;
    }
}

void RangeImageDisplay::loadTransformers()
{
    std::vector<std::string> classes = transformer_class_loader_->getDeclaredClasses();
    std::vector<std::string>::iterator ci;

    for (ci = classes.begin(); ci != classes.end(); ci++)
    {
        const std::string& lookup_name = *ci;
        std::string name = transformer_class_loader_->getName(lookup_name);

        if (transformers_.count(name) > 0)
        {
            ROS_ERROR("Transformer type [%s] is already loaded.", name.c_str());
            continue;
        }

        PointCloudTransformerPtr trans(transformer_class_loader_->createUnmanagedInstance(lookup_name));
        trans->init();
        connect(trans.get(), SIGNAL(needRetransform()), this, SLOT(causeRetransform()));

        TransformerInfo info;
        info.transformer = trans;
        info.readable_name = name;
        info.lookup_name = lookup_name;

        info.transformer->createProperties(this, PointCloudTransformer::Support_Color, info.color_props);
        setPropertiesHidden(info.color_props, true);

        transformers_[name] = info;
    }
}

void RangeImageDisplay::causeRetransform()
{
    needs_retransform_ = true;
}

void RangeImageDisplay::setPropertiesHidden(const QList<Property*>& props, bool hide)
{
    for (auto prop : props)
    {
        prop->setHidden(hide);
    }
}

void RangeImageDisplay::updateTransformers(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    std::string color_name = color_transformer_property_->getStdString();

    color_transformer_property_->clearOptions();

    // Get the channels that we could potentially render
    typedef std::set<std::pair<uint8_t, std::string>> S_string;
    S_string valid_color;
    bool cur_color_valid = false;
    bool has_rgb_transformer = false;
    auto trans_it = transformers_.begin();
    auto trans_end = transformers_.end();
    for (; trans_it != trans_end; ++trans_it)
    {
        const std::string& name = trans_it->first;
        const PointCloudTransformerPtr& trans = trans_it->second.transformer;
        uint32_t mask = trans->supports(cloud);

        if (mask & PointCloudTransformer::Support_Color)
        {
            valid_color.insert(std::make_pair(trans->score(cloud), name));
            if (name == color_name)
            {
                cur_color_valid = true;
            }
            if (name == "RGB8")
            {
                has_rgb_transformer = true;
            }
            color_transformer_property_->addOptionStd(name);
        }
    }

    if (!cur_color_valid)
    {
        if (!valid_color.empty())
        {
            if (has_rgb_transformer)
            {
                color_transformer_property_->setStringStd("RGB8");
            }
            else
            {
                color_transformer_property_->setStringStd(valid_color.rbegin()->second);
            }
        }
    }
}

void RangeImageDisplay::updateColorTransformer()
{
    if (transformers_.count(color_transformer_property_->getStdString()) == 0)
    {
        return;
    }
    new_color_transformer_ = true;
    causeRetransform();
}

PointCloudTransformerPtr RangeImageDisplay::getColorTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    auto it = transformers_.find(color_transformer_property_->getStdString());
    if (it != transformers_.end())
    {
        const PointCloudTransformerPtr& trans = it->second.transformer;
        if (trans->supports(cloud) & PointCloudTransformer::Support_Color)
        {
            return trans;
        }
    }

    return {};
}

void RangeImageDisplay::setColorTransformerOptions(EnumProperty* prop)
{
    fillTransformerOptions(prop, PointCloudTransformer::Support_Color);
}

void RangeImageDisplay::fillTransformerOptions(EnumProperty* prop, uint32_t mask)
{
    prop->clearOptions();

    if (existing_columns_.empty())
        return;

    auto it = transformers_.begin();
    for (; it != transformers_.end(); ++it)
    {
        const PointCloudTransformerPtr& trans = it->second.transformer;
        if ((trans->supports(existing_columns_.back()) & mask) == mask)
        {
            prop->addOption(QString::fromStdString(it->first));
        }
    }
}

void RangeImageDisplay::transformClouds()
{
    if (existing_columns_.empty())
        return;

    if (!needs_retransform_ && !new_message_available)
        return;
    needs_retransform_ = false;
    new_message_available = false;

    // create single message based on columns (important e.g. for min/max of intensity color transformer)
    auto& first_column = existing_columns_.front();
    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header = first_column->header;
    msg->height = first_column->height;
    msg->width = existing_columns_.size();
    msg->fields = first_column->fields;
    msg->is_bigendian = first_column->is_bigendian;
    msg->point_step = first_column->point_step;
    msg->row_step = first_column->point_step * msg->width;
    msg->is_dense = first_column->is_dense;
    msg->data.resize(msg->width * msg->height * msg->point_step);
    auto column_it = existing_columns_.begin();
    for (int column_index = 0; column_index < msg->width; column_index++)
    {
        uint8_t* byte_index_dst = msg->data.data() + (column_index * msg->point_step);
        uint8_t* byte_index_src = (*column_it)->data.data();
        for (int row_index = 0; row_index < msg->height; row_index++)
        {
            memcpy(byte_index_dst, byte_index_src, msg->point_step);
            byte_index_dst += msg->row_step;
            byte_index_src += msg->point_step;
        }
        column_it++;
    }

    // get the appropriate color transformer for these clouds (we assume all clouds have the same type)
    PointCloudTransformerPtr color_trans = getColorTransformer(existing_columns_.front());
    if (!color_trans)
    {
        std::stringstream ss;
        ss << "No color transformer available for cloud";
        setStatusStd(StatusProperty::Error, "Message", ss.str());
        return;
    }

    // dummy tf required below
    Ogre::Matrix4 tf;
    tf.makeTransform(Ogre::Vector3(0, 0, 0), Ogre::Vector3(1, 1, 1), Ogre::Quaternion(1, 0, 0, 0));

    PointCloud::Point default_pt;
    default_pt.color = Ogre::ColourValue(0, 1, 0);
    default_pt.position = Ogre::Vector3::ZERO;
    RangeImageTexture::CloudPoints colorized_points(msg->width * msg->height, default_pt);

    color_trans->transform(msg, PointCloudTransformer::Support_Color, tf, colorized_points);

    texture_.setColorForNanPoints(mark_nan_pixels_property_->getBool(), nan_color_property_->getOgreColor());
    texture_.generateNewImage(msg, colorized_points);
}

void RangeImageDisplay::resetForFlip()
{
    texture_.clear();
    existing_columns_.clear();
    render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::RangeImageDisplay, rviz::Display)
