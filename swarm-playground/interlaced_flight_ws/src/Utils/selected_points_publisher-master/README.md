# RViz plugin for publishing selected points

This plugin can be used to select points in RViz, and publish them to a new topic.

# Install
Build as per and normal ROS package. The plugin can be accessed after install by opening RViz add adding within the '+' menu from the 'Tools' panel (Panels->Tools).

## Usage
Drag with the left button to select objects in the 3D scene.
* Hold the Alt key to change viewpoint as in the Move tool
* Hold the Shift key to add more poiints to the current selection
* Hold the Ctrl key to remove points from the current selection

The following keys are also available:
* 'S' - Toggle with RViz interaction
* 'C' - Clear current selection
* 'P' - Publish selected points to /rviz_selected_points
