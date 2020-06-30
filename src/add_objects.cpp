#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_ros_api_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }

  moveit_msgs::PlanningScene planning_scene;

  // ground
  {
    moveit_msgs::AttachedCollisionObject ground;
    ground.link_name = "ground_object";
    ground.object.header.frame_id = "world";
    ground.object.id = "box1";

    geometry_msgs::Pose pose;
    pose.position.z = -0.05;
    pose.orientation.w = 1.0;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2;
    primitive.dimensions[1] = 1;
    primitive.dimensions[2] = 0.1;

    ground.object.primitives.push_back(primitive);
    ground.object.primitive_poses.push_back(pose);

    ground.object.operation = ground.object.ADD;

    planning_scene.world.collision_objects.push_back(ground.object);
  }

  // right wall
  {
    moveit_msgs::AttachedCollisionObject right_wall;
    right_wall.link_name = "left_wall_object";
    right_wall.object.header.frame_id = "world";
    right_wall.object.id = "box2";

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = -0.5 - 0.05;
    pose.position.z = 0.5;
    pose.orientation.w = 1.0;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 1;

    right_wall.object.primitives.push_back(primitive);
    right_wall.object.primitive_poses.push_back(pose);

    right_wall.object.operation = right_wall.object.ADD;

    planning_scene.world.collision_objects.push_back(right_wall.object);
  }

  { // left wall
    moveit_msgs::AttachedCollisionObject left_wall;
    left_wall.link_name = "left_wall_object";
    left_wall.object.header.frame_id = "world";
    left_wall.object.id = "box3";

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0.5 + 0.05;
    pose.position.z = 0.5;
    pose.orientation.w = 1.0;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 1;

    left_wall.object.primitives.push_back(primitive);
    left_wall.object.primitive_poses.push_back(pose);

    left_wall.object.operation = left_wall.object.ADD;

    planning_scene.world.collision_objects.push_back(left_wall.object);
  }

  { // back wall
    moveit_msgs::AttachedCollisionObject back_wall;
    back_wall.link_name = "back_wall_object";
    back_wall.object.header.frame_id = "world";
    back_wall.object.id = "box4";

    geometry_msgs::Pose pose;
    pose.position.x = -0.5 - 0.05;
    pose.position.y = 0;
    pose.position.z = 0.5;
    pose.orientation.w = 1.0;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 1;
    primitive.dimensions[2] = 1;

    back_wall.object.primitives.push_back(primitive);
    back_wall.object.primitive_poses.push_back(pose);

    back_wall.object.operation = back_wall.object.ADD;

    planning_scene.world.collision_objects.push_back(back_wall.object);
  }

  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);


  ros::shutdown();
  return 0;
}
