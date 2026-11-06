#include <QApplication>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <OgreEntity.h>
#include <OgreSubEntity.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>
#include "advrf_ros2_io/selected_joint_robot.hpp"
#include <rviz_rendering/render_system.hpp>
class TestDisplay : public advrf_ros2_io::SelectedJointRobot {
public:
  using SelectedJointRobot::updateHighlight;
  using RobotModelDisplay::load_urdf_from_string;
  bool empty() {return robot_->getLinks().empty();}
  void normalMaterials() {for (const auto & entry : robot_->getLinks()) entry.second->unsetColor();}
  Ogre::ColourValue color(const std::string & name) {
    return robot_->getLink(name)->getVisualMeshes().at(0)->getSubEntity(0)->getMaterial()->getTechnique(0)->getPass(0)->getDiffuse();
  }
};
void require(bool condition, const char * message) {if (!condition) throw std::runtime_error(message);}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  QApplication app(argc,argv);
  {
    pluginlib::ClassLoader<rviz_common::Display> loader("rviz_common", "rviz_common::Display");
    auto loaded = loader.createSharedInstance("advrf_ros2_io/SelectedJointRobot");
    require(loaded->subProp("Selection Topic")->getValue().toString()=="inputs/selected_joint", "plugin property missing");
    auto ros = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("highlight_test");
    rviz_rendering::RenderSystem::get();
    rviz_common::VisualizationFrame frame(ros);
    frame.setApp(&app);
    frame.setSplashPath("");
    frame.initialize(ros);
    auto & manager = *frame.getManager();
    manager.startUpdate();
    TestDisplay display;
    display.initialize(&manager);
    display.setEnabled(true);
    display.load_urdf_from_string(R"(<robot name="test">
<link name="base"/>
<link name="one"><visual><geometry><box size="0.1 0.1 0.1"/></geometry><material name="green"><color rgba="0 1 0 1"/></material></visual></link>
<link name="two"><visual><geometry><box size="0.1 0.1 0.1"/></geometry><material name="blue"><color rgba="0 0 1 1"/></material></visual></link>
<joint name="j1" type="continuous"><parent link="base"/><child link="one"/><axis xyz="0 0 1"/></joint>
<joint name="j2" type="continuous"><parent link="one"/><child link="two"/><axis xyz="0 0 1"/></joint>
</robot>)");
    display.normalMaterials();
    const auto original1=display.color("one"), original2=display.color("two");
    auto pub=ros->get_raw_node()->create_publisher<sensor_msgs::msg::JointState>("inputs/selected_joint",rclcpp::QoS(1).transient_local());
    auto select=[&](std::string joint) {
      sensor_msgs::msg::JointState msg; if (!joint.empty()) msg.name={joint}; pub->publish(msg);
      for(int i=0;i<30;++i) {app.processEvents(); std::this_thread::sleep_for(std::chrono::milliseconds(10));}
      display.updateHighlight();
    };
    select("j1");
    require(display.color("one")!=original1, "first joint was not highlighted");
    require(display.color("two")==original2, "unselected link changed");
    select("j2");
    require(display.color("one")==original1, "first material not restored");
    require(display.color("two")!=original2, "second joint was not highlighted");
    select("unknown"); require(display.color("two")==original2,"unknown selection did not restore material");
    select("j1"); display.setEnabled(false);
    require(display.empty(),"disabled RobotModel should clear its loaded links");
    std::cout<<"Plugin load, link highlighting, selection changes and material restoration passed\n";
  }
  rclcpp::shutdown();
}
