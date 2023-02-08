/*
 * Copyright 2019-2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v0.1.0: drwnz (david.wong@tier4.jp) *
 *
 * selected_points_publisher.cpp
 *
 *  Created on: December 5th 2019
 */

#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/view_manager.h"
#include "rviz/view_controller.h"
#include "rviz/geometry.h"
#include "OGRE/OgreCamera.h"

#include "selected_points_publisher/selected_points_publisher.hpp"

#include <ros/ros.h>
#include <ros/time.h>
#include <QVariant>
#include <visualization_msgs/Marker.h>

namespace rviz_plugin_selected_points_publisher
{
  SelectedPointsPublisher::SelectedPointsPublisher()
  {
    updateTopic();
  }

  SelectedPointsPublisher::~SelectedPointsPublisher()
  {
  }

  void SelectedPointsPublisher::updateTopic()
  {
    node_handle_.param("frame_id", tf_frame_, std::string("/base_link"));
    selected_drones_topic_ = std::string("/rviz_selected_drones");
    goal_topic_ = std::string("/goal");

    rviz_selected_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(selected_drones_topic_.c_str(), 100);
    goal_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(goal_topic_.c_str(), 100);
    num_selected_points_ = 0;
  }

  int SelectedPointsPublisher::processKeyEvent(QKeyEvent *event, rviz::RenderPanel *panel)
  {
    int flags = rviz::SelectionTool::processKeyEvent(event, panel);
    if (event->type() == QKeyEvent::KeyPress)
    {
      if (event->key() == 'c' || event->key() == 'C')
      {
        ROS_INFO_STREAM_NAMED("SelectedPointsPublisher::processKeyEvent", "Cleaning previous selection (selected area "
                                                                          "and points).");
        rviz::SelectionManager *selection_manager = context_->getSelectionManager();
        rviz::M_Picked selection = selection_manager->getSelection();
        selection_manager->removeSelection(selection);
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::DELETE;
        marker.lifetime = ros::Duration();
        num_selected_points_ = 0;
      }
      else if (event->key() == 'p' || event->key() == 'P')
      {
        ROS_INFO_STREAM_NAMED("SelectedPointsPublisher.updateTopic",
                              "Publishing " << num_selected_points_ << " selected points to topic "
                                            << node_handle_.resolveName(selected_drones_topic_));
        //
        for (int i = 0; i < selected_drones_.size(); ++i)
        {
          rviz_selected_publisher_.publish(selected_drones_[i]);
        }
      }
    }

    return flags;
  }

  int SelectedPointsPublisher::processMouseEvent(rviz::ViewportMouseEvent &event)
  {
    int flags = rviz::SelectionTool::processMouseEvent(event);
    if (event.alt())
    {
      selecting_ = false;
    }
    else
    {
      if (event.leftDown())
      {
        selecting_ = true;
      }
      if (event.rightUp()) // Publish immediately!
      {
        Ogre::Vector3 intersection;
        Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
        if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x,
                                              event.y, intersection))
        {
          this->processSelectedArea();
          for (int i = 0; i < selected_drones_.size(); ++i)
          {
            rviz_selected_publisher_.publish(selected_drones_[i]);
          }

          ros::Duration(0.1).sleep();
          
          geometry_msgs::PoseStamped goal_msg;
          goal_msg.header.frame_id = std::string("world");
          goal_msg.header.stamp = ros::Time::now();
          goal_msg.pose.position.x = intersection.x;
          goal_msg.pose.position.y = intersection.y;
          goal_msg.pose.position.z = 1.0;
          goal_publisher_.publish(goal_msg);
        }
      }
    }

    if (selecting_)
    {
      if (event.leftUp())
      {
        this->processSelectedArea();
      }
    }
    return flags;
  }

  int SelectedPointsPublisher::processSelectedArea()
  {
    rviz::SelectionManager *selection_manager = context_->getSelectionManager();
    rviz::M_Picked selection = selection_manager->getSelection();
    rviz::PropertyTreeModel *model = selection_manager->getPropertyModel();

    selected_drones_.clear();
    int i = 0;
    while (model->hasIndex(i, 0))
    {

      QModelIndex child_index = model->index(i, 0);

      rviz::Property *child = model->getProp(child_index);
      rviz::VectorProperty *subchild = (rviz::VectorProperty *)child->childAt(0);
      Ogre::Vector3 point_data = subchild->getVector();
      // ROS_ERROR("%f %f %f", point_data.x, point_data.y, point_data.z);
      std::string name = child->getNameStd();
      // ROS_ERROR("%s", name.c_str());

      if (name.substr(0, 12) == std::string("Marker drone"))
      {
        geometry_msgs::PoseStamped droneX_msg;
        droneX_msg.header.frame_id = std::string("drone_") + name.substr(13, 20);
        droneX_msg.header.stamp = ros::Time::now();
        droneX_msg.pose.position.x = point_data.x;
        droneX_msg.pose.position.y = point_data.y;
        droneX_msg.pose.position.z = point_data.z;

        selected_drones_.push_back(droneX_msg);
      }

      i++;
    }
    num_selected_points_ = i;
    ROS_INFO_STREAM_NAMED("SelectedPointsPublisher._processSelectedAreaAndFindPoints",
                          "Number of points in the selected area: " << num_selected_points_);

    return 0;
  }
} // namespace rviz_plugin_selected_points_publisher

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_selected_points_publisher::SelectedPointsPublisher, rviz::Tool)
