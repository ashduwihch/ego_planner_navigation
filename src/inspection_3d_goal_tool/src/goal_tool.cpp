#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <OgreCamera.h>
#include <OgreColourValue.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include <QIcon>
#include <QString>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/render_window.hpp>

namespace inspection_3d_goal_tool
{

class Goal3DTool final : public rviz_common::Tool
{
public:
  Goal3DTool()
  {
    shortcut_key_ = 'g';
    topic_property_ = new rviz_common::properties::StringProperty(
      "Goal topic", "/goal_pose", "PoseStamped topic received by EGO-Planner",
      getPropertyContainer());
    height_property_ = new rviz_common::properties::FloatProperty(
      "Height (m)", 0.5f,
      "Initial height; hold left, then hold right and drag vertically to adjust",
      getPropertyContainer());
    height_property_->setMin(0.1f);
    height_property_->setMax(3.8f);
  }

  ~Goal3DTool() override
  {
    if (label_node_) {
      label_node_->detachObject(label_.get());
      scene_manager_->destroySceneNode(label_node_);
    }
  }

  void onInitialize() override
  {
    setName("3D Nav Goal");
    setDescription(
      "Hold left to select XY, hold right and drag up/down for height, release left to send.");
    setIcon(QIcon(QString::fromStdString(
      ament_index_cpp::get_package_share_directory("inspection_3d_goal_tool") +
      "/icons/3d_goal.svg")));
    target_marker_ = std::make_unique<rviz_rendering::Shape>(
      rviz_rendering::Shape::Sphere, scene_manager_, nullptr);
    target_marker_->setColor(0.05f, 0.75f, 0.25f, 0.95f);
    target_marker_->setScale(Ogre::Vector3(0.22f, 0.22f, 0.22f));
    ground_marker_ = std::make_unique<rviz_rendering::Shape>(
      rviz_rendering::Shape::Sphere, scene_manager_, nullptr);
    ground_marker_->setColor(1.0f, 0.55f, 0.0f, 0.95f);
    ground_marker_->setScale(Ogre::Vector3(0.13f, 0.13f, 0.13f));
    height_line_ = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_);
    height_line_->setLineWidth(0.025f);
    height_line_->setNumLines(1);
    height_line_->setMaxPointsPerLine(2);
    label_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    label_ = std::make_unique<rviz_rendering::MovableText>(
      "", "Liberation Sans", 0.16f, Ogre::ColourValue(0.05f, 0.12f, 0.18f, 1.0f));
    label_->showOnTop(true);
    label_node_->attachObject(label_.get());
    hidePreview();
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    updatePublisher();
  }

  void activate() override
  {
    selected_ = false;
    setStatus("Hold left to select XY; while holding left, hold right and drag for Z; release left to send.");
  }

  void deactivate() override
  {
    selected_ = false;
    adjusting_height_ = false;
    hidePreview();
  }

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override
  {
    if (event.leftDown()) {
      if (context_->getFixedFrame() != "lidar") {
        setStatus("Set RViz Fixed Frame to lidar before selecting a 3D goal.");
        return Render;
      }
      Ogre::Vector3 ground;
      if (!pointOnGround(event, ground)) {
        setStatus("Could not intersect the horizontal goal plane from this camera view.");
        return Render;
      }
      selected_point_ = ground;
      selected_point_.z = height_property_->getFloat();
      selected_ = true;
      adjusting_height_ = false;
      height_selected_ = false;
      showPreview(selected_point_);
      setStatus(positionStatus("XY fixed. Keep left held; right-drag up/down for Z."));
      return Render;
    }

    if (selected_ && event.rightDown() && event.left()) {
      adjusting_height_ = true;
      height_selected_ = true;
      last_drag_y_ = event.y;
      setStatus(positionStatus("Drag up/down for height; release left to send."));
      return Render;
    }

    if (selected_ && event.type == QEvent::MouseMove &&
        adjusting_height_ && event.left() && event.right()) {
      constexpr float meters_per_pixel = 0.0125f;
      const float height = std::clamp(
        selected_point_.z - (event.y - last_drag_y_) * meters_per_pixel,
        0.1f, 3.8f);
      last_drag_y_ = event.y;
      selected_point_.z = height;
      height_property_->setFloat(height);
      showPreview(selected_point_);
      setStatus(positionStatus("Release left to send this 3D goal."));
      return Render;
    }

    if (selected_ && event.rightUp()) {
      adjusting_height_ = false;
      setStatus(positionStatus("Height selected. Release left to send."));
      return Render;
    }

    if (selected_ && event.leftUp()) {
      selected_ = false;
      adjusting_height_ = false;
      hidePreview();
      if (!height_selected_) {
        setStatus("Goal cancelled: hold left, then use right-drag to select height.");
        return Render;
      }
      if (context_->getFixedFrame() != "lidar") {
        setStatus("Goal cancelled: RViz Fixed Frame must remain lidar.");
        return Render;
      }
      updatePublisher();
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = "lidar";
      goal.header.stamp = context_->getClock()->now();
      goal.pose.position.x = selected_point_.x;
      goal.pose.position.y = selected_point_.y;
      goal.pose.position.z = selected_point_.z;
      goal.pose.orientation.w = 1.0;
      publisher_->publish(goal);
      setStatus(positionStatus("3D goal sent."));
      return Finished | Render;
    }

    if (!selected_ && event.type == QEvent::MouseMove) {
      Ogre::Vector3 hover;
      if (pointOnGround(event, hover)) {
        hover.z = height_property_->getFloat();
        showPreview(hover);
      } else {
        hidePreview();
      }
      return Render;
    }
    return Render;
  }

private:
  bool pointOnGround(
    const rviz_common::ViewportMouseEvent & event, Ogre::Vector3 & point) const
  {
    auto * window = event.panel->getRenderWindow();
    auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(window);
    if (!viewport || viewport->getActualWidth() == 0 || viewport->getActualHeight() == 0) {
      return false;
    }
    const double pixel_ratio = std::max(1, event.device_pixel_ratio);
    const double u = event.x * pixel_ratio / viewport->getActualWidth();
    const double v = event.y * pixel_ratio / viewport->getActualHeight();
    const Ogre::Ray ray = viewport->getCamera()->getCameraToViewportRay(u, v);
    const Ogre::Plane plane(Ogre::Vector3::UNIT_Z, Ogre::Vector3::ZERO);
    const auto hit = ray.intersects(plane);
    if (!hit.first || hit.second < 0 || !std::isfinite(hit.second)) {
      return false;
    }
    point = ray.getPoint(hit.second);
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  }

  void showPreview(const Ogre::Vector3 & target)
  {
    const Ogre::Vector3 ground(target.x, target.y, 0.0f);
    target_marker_->setPosition(target);
    target_marker_->getRootNode()->setVisible(true);
    ground_marker_->setPosition(ground);
    ground_marker_->getRootNode()->setVisible(true);
    height_line_->clear();
    height_line_->addPoint(ground, Ogre::ColourValue(1.0f, 0.55f, 0.0f, 0.95f));
    height_line_->addPoint(target, Ogre::ColourValue(0.05f, 0.75f, 0.25f, 0.95f));
    height_line_->finishLine();
    height_line_->getSceneNode()->setVisible(true);
    const QString caption = QString("x %1   y %2   z %3 m")
      .arg(target.x, 0, 'f', 2).arg(target.y, 0, 'f', 2).arg(target.z, 0, 'f', 2);
    label_->setCaption(caption.toStdString());
    label_node_->setPosition(target + Ogre::Vector3(0.0f, 0.0f, 0.23f));
    label_node_->setVisible(true);
  }

  void hidePreview()
  {
    target_marker_->getRootNode()->setVisible(false);
    ground_marker_->getRootNode()->setVisible(false);
    height_line_->getSceneNode()->setVisible(false);
    label_node_->setVisible(false);
  }

  QString positionStatus(const QString & instruction) const
  {
    return QString("x %1, y %2, z %3 m. %4")
      .arg(selected_point_.x, 0, 'f', 2)
      .arg(selected_point_.y, 0, 'f', 2)
      .arg(selected_point_.z, 0, 'f', 2)
      .arg(instruction);
  }

  void updatePublisher()
  {
    const std::string topic = topic_property_->getStdString();
    if (!publisher_ || topic != published_topic_) {
      publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
      published_topic_ = topic;
    }
  }

  rviz_common::properties::StringProperty * topic_property_{};
  rviz_common::properties::FloatProperty * height_property_{};
  std::unique_ptr<rviz_rendering::Shape> target_marker_;
  std::unique_ptr<rviz_rendering::Shape> ground_marker_;
  std::unique_ptr<rviz_rendering::BillboardLine> height_line_;
  std::unique_ptr<rviz_rendering::MovableText> label_;
  Ogre::SceneNode * label_node_{};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  std::string published_topic_;
  Ogre::Vector3 selected_point_{Ogre::Vector3::ZERO};
  bool selected_{false};
  bool adjusting_height_{false};
  bool height_selected_{false};
  int last_drag_y_{0};
};

}  // namespace inspection_3d_goal_tool

PLUGINLIB_EXPORT_CLASS(inspection_3d_goal_tool::Goal3DTool, rviz_common::Tool)
