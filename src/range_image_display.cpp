#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreTextureManager.h>
#include <boost/bind.hpp>

#include <rviz/default_plugin/point_cloud_transformer.h>
#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/compatibility.h>
#include <rviz/render_panel.h>
#include <rviz/validate_floats.h>

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

    flip_property_ = new BoolProperty("Flip",
                                      false,
                                      "Flips width and height (rotate 90° degrees counterclockwise).",
                                      this,
                                      SLOT(causeRetransform()),
                                      this);

    keep_aspect_ratio_property_ = new BoolProperty("Keep Aspect Ratio",
                                                   true,
                                                   "Keep the aspect ratio of range image while resizing window.",
                                                   this,
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
            sub_ = update_nh_.subscribe(topic_property_->getTopicStd(), 5, &RangeImageDisplay::incomingMessage, this);
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

    if (needs_retransform_)
    {
        colorizeCloudAndAddToTexture();
        needs_retransform_ = false;
    }

    texture_.update();

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
    last_msg_ = msg;
    updateTransformers(msg);
    colorizeCloudAndAddToTexture();
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

    if (!last_msg_)
    {
        return;
    }

    auto it = transformers_.begin();
    for (; it != transformers_.end(); ++it)
    {
        const PointCloudTransformerPtr& trans = it->second.transformer;
        if ((trans->supports(last_msg_) & mask) == mask)
        {
            prop->addOption(QString::fromStdString(it->first));
        }
    }
}

void RangeImageDisplay::colorizeCloudAndAddToTexture()
{
    if(!last_msg_)
        return;

    PointCloudTransformerPtr color_trans = getColorTransformer(last_msg_);

    if (!color_trans)
    {
        std::stringstream ss;
        ss << "No color transformer available for cloud";
        setStatusStd(StatusProperty::Error, "Message", ss.str());
        return;
    }

    Ogre::Matrix4 transform;
    transform.makeTransform(Ogre::Vector3(0, 0, 0), Ogre::Vector3(1, 1, 1), Ogre::Quaternion(1, 0, 0, 0)); // TODO

    PointCloud::Point default_pt;
    default_pt.color = Ogre::ColourValue(1, 1, 1);
    default_pt.position = Ogre::Vector3::ZERO;
    RangeImageTexture::CloudPointsPtr points(new RangeImageTexture::CloudPoints());
    points->resize(last_msg_->width * last_msg_->height, default_pt);

    color_trans->transform(last_msg_, PointCloudTransformer::Support_Color, transform, *points);

    texture_.addPoints(last_msg_->width, last_msg_->height, points);
    texture_.flipped(flip_property_->getBool());
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::RangeImageDisplay, rviz::Display)
