# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rqt_py_common.ini_helper import pack, unpack

import numpy as np
import random
import math as m

# http://github.com/ros2/geometry2/issues/701
import tf2_geometry_msgs # noqa: F401

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import (
    PointStamped,
    PolygonStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
)

from python_qt_binding.QtCore import Signal, Slot, QPointF, qWarning, Qt
from python_qt_binding.QtGui import (
    QColor,
    QImage,
    QPainterPath,
    QPen,
    QPixmap,
    QPolygonF,
    qRgb,
    QTransform,
)
from python_qt_binding.QtWidgets import (
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QInputDialog,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from rqt_py_common.topic_helpers import get_field_type

from .list_dialog import ListDialog


def quaternion_from_yaw(yaw):
    # quaternion order: x, y, z, w
    return (
        0.0,
        0.0,
        m.sin(yaw / 2.0),
        m.cos(yaw / 2.0),
    )


def accepted_topic(topic):
    msg_types = [OccupancyGrid, Path, PolygonStamped, PointStamped]
    msg_type, array = get_field_type(topic)

    if not array and msg_type in msg_types:
        return True
    else:
        return False

class PathInfo(object):
    def __init__(self, name=None):
        self.color = None
        self.sub = None
        self.cb = None
        self.path = None
        self.item = None
        self.name = name


class NavViewWidget(QWidget):
    def __init__(
        self,
        node,
        map_topic="/map",
        path_topics=None,
        footprint_topics=None,
    ):
        super(NavViewWidget, self).__init__()

        if path_topics is None:
            path_topics = []
        if footprint_topics is None:
            footprint_topics = []

        self._node = node
        self._logger = self._node.get_logger()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

        self._layout = QVBoxLayout()
        self._button_layout = QHBoxLayout()

        self.setAcceptDrops(True)
        self.setWindowTitle("Navigation Viewer")

        self._path_topics = path_topics
        self._footprint_topics = footprint_topics
        self._map_topic = map_topic

        self._set_pose = QPushButton("Set pose estimation")
        self._set_goal = QPushButton("Set goal pose")

        self._button_layout.addWidget(self._set_pose)
        self._button_layout.addWidget(self._set_goal)

        self._layout.addLayout(self._button_layout)

        self._nav_view = None

        self.setLayout(self._layout)

    def new_nav_view(self):
        if self._nav_view:
            self._nav_view.close()
        self._nav_view = NavView(
            self._node,
            self._tf_buffer,
            self._map_topic,
            self._path_topics,
            self._footprint_topics,
            tf_listener=self._tf_listener,
            parent=self,
        )
        self._set_pose.clicked.connect(self._nav_view.pose_mode)
        self._set_goal.clicked.connect(self._nav_view.goal_mode)
        self._layout.addWidget(self._nav_view)

    def dragEnterEvent(self, e):
        if not e.mimeData().hasText():
            if (
                not hasattr(e.source(), "selectedItems")
                or len(e.source().selectedItems()) == 0
            ):
                qWarning(
                    "NavView.dragEnterEvent(): not hasattr(event.source(), selectedItems) or "
                    "len(event.source().selectedItems()) == 0"
                )
                return
            item = e.source().selectedItems()[0]
            topic_name = item.data(0, Qt.UserRole)
            if topic_name is None:
                qWarning("NavView.dragEnterEvent(): not hasattr(item, ros_topic_name_)")
                return

        else:
            topic_name = str(e.mimeData().text())

        if accepted_topic(topic_name):
            e.acceptProposedAction()

    def dropEvent(self, e):
        if e.mimeData().hasText():
            topic_name = str(e.mimeData().text())
        else:
            dropped_item = e.source().selectedItems()[0]
            topic_name = str(dropped_item.data(0, Qt.UserRole))

        topic_type, array = get_field_type(topic_name)
        if not array:
            if topic_type is OccupancyGrid:
                self._map_topic = topic_name

                # Swap out the nav view for one with the new topics
                self.new_nav_view()
            elif topic_type is Path:
                self._path_topics.append(topic_name)
                self._nav_view.add_path(topic_name)
            elif topic_type is PolygonStamped:
                self._footprint_topics.append(topic_name)
                self._nav_view.add_polygon(topic_name)

    def save_settings(self, plugin_settings, instance_settings):
        self._logger.info("Saving settings: map_topic={}, paths={}, footprints={}".format(
            self._map_topic,
            self._path_topics,
            self._footprint_topics,
        ))
        instance_settings.set_value("map_topic", self._map_topic)
        instance_settings.set_value("paths", pack(self._path_topics))
        instance_settings.set_value("footprints", pack(self._footprint_topics))

    def restore_settings(self, plugin_settings, instance_settings):
        try:
            self._map_topic = instance_settings.value("map_topic", "/map")
        except Exception:
            pass

        try:
            self._path_topics = unpack(instance_settings.value("paths", []))
        except Exception:
            pass

        try:
            self._footprint_topics = unpack(instance_settings.value("footprints", []))
        except Exception:
            pass

        self._logger.info("Restoring settings: map_topic={}, paths={}, footprints={}".format(
            self._map_topic,
            self._path_topics,
            self._footprint_topics,
        ))
        self.new_nav_view()

    def find_topic_by_type(self, topic_type):
        """
        Find a topic by its type
        :param topic_type: Type of the topic to find
        :return: Topic name or None if not found
        """
        topics = [
            topic
            for topic, types in self._node.get_topic_names_and_types()
            if topic_type in types
        ]
        print("Found topics of type {}: {}".format(topic_type, topics))
        return topics

    def trigger_configuration(self):
        """
        Callback when the configuration button is clicked
        """
        changed = False
        map_topics = sorted(self.find_topic_by_type("nav_msgs/msg/OccupancyGrid"))
        try:
            index = map_topics.index(self._map_topic)
        except ValueError:
            index = 0
        map_topic, ok = QInputDialog.getItem(
            self, "Select map topic name", "Topic name", map_topics, index
        )
        if ok:
            if map_topic != self._map_topic:
                changed = True
            self._map_topic = map_topic

        # Paths
        path_topics = sorted(self.find_topic_by_type("nav_msgs/msg/Path"))
        path_topics = [(topic, topic in self._path_topics) for topic in path_topics]
        dialog = ListDialog("Select path topic(s)", path_topics, self)
        paths, ok = dialog.exec_()

        if ok:
            if not paths:
                changed = True
            diff = set(paths).symmetric_difference(set(self._path_topics))
            if diff:
                self._path_topics = paths
                changed = True

        # Polygons
        polygon_topics = sorted(self.find_topic_by_type("geometry_msgs/msg/PolygonStamped"))
        polygon_topics = [
            (topic, topic in self._footprint_topics) for topic in polygon_topics
        ]
        dialog = ListDialog("Select polygon topic(s)", polygon_topics, self)
        footprints, ok = dialog.exec_()

        if ok:
            if not footprints:
                changed = True
            diff = set(footprints).symmetric_difference(set(self._footprint_topics))
            if diff:
                self._footprint_topics = footprints
                changed = True

        if changed:
            self._logger.debug(
                "New configuration is different, creating a new nav_view"
            )
            self.new_nav_view()


class NavView(QGraphicsView):
    map_changed = Signal()
    path_changed = Signal(str)
    polygon_changed = Signal(str)

    def __init__(
        self,
        node,
        tf_buffer,
        map_topic,
        path_topics,
        footprint_topics,
        tf_listener,
        parent,
    ):
        super(NavView, self).__init__()

        self._node = node
        self._tf_buffer = tf_buffer
        self._parent = parent
        self._tf_listener = tf_listener

        self._pose_mode = False
        self._goal_mode = False

        self._latest_path = None
        self._drag_start = None

        self.map_changed.connect(self._update)
        self.destroyed.connect(self.close)

        # ScrollHandDrag
        self.setDragMode(QGraphicsView.ScrollHandDrag)

        self._map = None
        self._map_item = None

        self._map_width = 0
        self._map_height = 0
        self._map_resolution = 0
        self._map_origin = None
        self._frame_id = ""

        self._paths = {}
        self._polygons = {}
        self.path_changed.connect(self._update_path)
        self.polygon_changed.connect(self._update_polygon)

        self._colors = [
            (238, 34, 116),
            (68, 134, 252),
            (236, 228, 46),
            (102, 224, 18),
            (242, 156, 6),
            (240, 64, 10),
            (196, 30, 250),
        ]

        self._scene = QGraphicsScene()

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.map_sub = self._node.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            qos_profile,
        )

        for path in path_topics:
            self.add_path(path)

        for poly in footprint_topics:
            self.add_polygon(poly)

        self._pose_pub = self._node.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            1,
        )
        self._goal_pub = self._node.create_publisher(
            PoseStamped,
            "/goal_pose",
            1,
        )

        self.setScene(self._scene)

    def add_dragdrop(self, item):
        # Add drag and drop functionality to all the items in the view
        def c(x, e):
            self.dragEnterEvent(e)

        def d(x, e):
            self.dropEvent(e)

        item.setAcceptDrops(True)
        item.dragEnterEvent = c
        item.dropEvent = d

    def dragEnterEvent(self, e):
        if self._parent:
            self._parent.dragEnterEvent(e)

    def dropEvent(self, e):
        if self._parent:
            self._parent.dropEvent(e)

    def wheelEvent(self, event):
        event.ignore()
        try:
            delta = event.angleDelta().y()
        except AttributeError:
            delta = event.delta()
        if delta > 0:
            self.scale(1.15, 1.15)
        else:
            self.scale(0.85, 0.85)

    def map_callback(self, msg):
        self._map_resolution = msg.info.resolution
        self._map_width = msg.info.width
        self._map_height = msg.info.height
        self._map_origin = msg.info.origin
        self._frame_id = msg.header.frame_id

        a = np.array(msg.data, dtype=np.uint8, copy=False, order="C")
        a = a.reshape((self._map_height, self._map_width))
        if self._map_width % 4:
            e = np.empty(
                (self._map_height, 4 - self._map_width % 4), dtype=a.dtype, order="C"
            )
            a = np.append(a, e, axis=1)
        image = QImage(
            a.reshape((a.shape[0] * a.shape[1])),
            self._map_width,
            self._map_height,
            QImage.Format_Indexed8,
        )

        for i in reversed(range(101)):
            image.setColor(100 - i, qRgb(*(int(x) for x in (i * 2.55, i * 2.55, i * 2.55))))
        image.setColor(101, qRgb(255, 0, 0))  # not used indices
        image.setColor(255, qRgb(200, 200, 200))  # color for unknown value -1
        self._map = image
        self.setSceneRect(0, 0, self._map_width, self._map_height)
        self.map_changed.emit()

    def add_path(self, name):
        path = PathInfo(name)

        def path_callback(msg):
            if not self._map:
                return

            pp = QPainterPath()

            # Transform everything in to the map frame
            if not (msg.header.frame_id == self._frame_id or msg.header.frame_id == ""):
                try:
                    data = [
                        self._tf_buffer.transform(
                            pose,
                            self._frame_id,
                            timeout=rclpy.duration.Duration(seconds=10),
                        )
                        for pose in msg.poses
                    ]
                except TransformException:
                    self._logger.error(
                        "Could not convert the {} frame to the map frame {}".format(
                            msg.header.frame_id, self._frame_id
                        )
                    )
                    data = []
            else:
                data = msg.poses

            if len(data) > 0:
                start = data[0].pose.position
                pp.moveTo(*self.point_map_to_qt((start.x, start.y)))

                for pose in data:
                    pt = pose.pose.position
                    pp.lineTo(*self.point_map_to_qt((pt.x, pt.y)))

                path.path = pp
                self.path_changed.emit(name)

        path.color = random.choice(self._colors)
        self._colors.remove(path.color)

        path.cb = path_callback
        path.sub = self._node.create_subscription(
            Path,
            name,
            path.cb,
            1,
        )

        self._paths[name] = path

    def add_polygon(self, name):
        poly = PathInfo(name)

        def polygon_callback(msg):
            if not self._map:
                return

            if not (msg.header.frame_id == self._frame_id or msg.header.frame_id == ""):
                try:
                    points_stamped = []
                    for pt in msg.polygon.points:
                        ps = PointStamped()
                        ps.header.frame_id = msg.header.frame_id
                        ps.point.x = pt.x
                        ps.point.y = pt.y

                        points_stamped.append(ps)

                    trans_pts = []
                    for pt in points_stamped:
                        point = self._tf_buffer.transform(
                            pt,
                            self._frame_id,
                            timeout=rclpy.duration.Duration(seconds=10),
                        ).point
                        trans_pts.append((point.x, point.y))
                except TransformException:
                    self._logger.error(
                        "Could not convert the {} frame to the map frame {}".format(
                            msg.header.frame_id, self._frame_id
                        )
                    )
                    trans_pts = []
            else:
                trans_pts = [(pt.x, pt.y) for pt in msg.polygon.points]

            if len(trans_pts) > 0:
                trans_pts.append(trans_pts[0])
                pts = [QPointF(*self.point_map_to_qt(pt)) for pt in trans_pts]
                poly.path = QPolygonF(pts)

                self.polygon_changed.emit(name)

        poly.color = random.choice(self._colors)
        self._colors.remove(poly.color)

        poly.cb = polygon_callback
        poly.sub = self._node.create_subscription(
            PolygonStamped,
            name,
            poly.cb,
            1,
        )

        self._polygons[name] = poly

    def pose_mode(self):
        if not self._pose_mode:
            self._pose_mode = True
            self._goal_mode = False
            self.setDragMode(QGraphicsView.NoDrag)
        elif self._pose_mode:
            self._pose_mode = False
            self.setDragMode(QGraphicsView.ScrollHandDrag)

    def goal_mode(self):
        if not self._goal_mode:
            self._goal_mode = True
            self._pose_mode = False
            self.setDragMode(QGraphicsView.NoDrag)
        elif self._goal_mode:
            self._goal_mode = False
            self.setDragMode(QGraphicsView.ScrollHandDrag)

    def draw_position(self, e):
        p = self.mapToScene(e.x(), e.y())
        v = (p.x() - self._drag_start[0], p.y() - self._drag_start[1])
        mag = m.sqrt(pow(v[0], 2) + pow(v[1], 2))
        if mag < 1e-6:
            self._node.get_logger().warning("NavView.draw_position(): Dragging too short, ignoring")
            return None, None
        v = (v[0] / mag, v[1] / mag)  # Normalize diff vector
        u = (-v[1], v[0])  # Project diff vector to mirrored map

        if self._latest_path:
            self._scene.removeItem(self._latest_path)
            self._latest_path = None

        res = (v[0] * 25, v[1] * 25)

        if self._pose_mode:
            pen = QPen(QColor("red"))
        elif self._goal_mode:
            pen = QPen(QColor("green"))
        self._latest_path = self._scene.addLine(
            self._drag_start[0],
            self._drag_start[1],
            self._drag_start[0] + res[0],
            self._drag_start[1] + res[1],
            pen,
        )

        map_p = self.point_qt_to_map(self._drag_start)

        angle = m.atan2(u[0], u[1])
        quat = quaternion_from_yaw(angle)

        self._drag_start = None

        return map_p, quat

    def mousePressEvent(self, e):
        if self._goal_mode or self._pose_mode:
            p = self.mapToScene(e.x(), e.y())
            self._drag_start = (p.x(), p.y())
        else:
            super(NavView, self).mousePressEvent(e)

    def mouseReleaseEvent(self, e):
        if self._goal_mode:
            map_p, quat = self.draw_position(e)
            if map_p is None or quat is None:
                return
            self.goal_mode()  # Disable goal_mode and enable dragging/scrolling again

            msg = PoseStamped()
            msg.header.frame_id = self._frame_id
            msg.header.stamp = self._node.get_clock().now().to_msg()

            msg.pose.position.x = map_p[0]
            msg.pose.position.y = map_p[1]
            msg.pose.orientation.z = quat[2]
            msg.pose.orientation.w = quat[3]

            self._goal_pub.publish(msg)

        elif self._pose_mode:
            map_p, quat = self.draw_position(e)
            if map_p is None or quat is None:
                return
            self.pose_mode()  # Disable pose_mode and enable dragging/scrolling again

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self._frame_id
            msg.header.stamp = self._node.get_clock().now().to_msg()

            # ToDo: Is it ok to just ignore the covariance matrix here?
            msg.pose.pose.orientation.z = quat[2]
            msg.pose.pose.orientation.w = quat[3]
            msg.pose.pose.position.x = map_p[0]
            msg.pose.pose.position.y = map_p[1]

            self._pose_pub.publish(msg)

    def close(self):
        if self.map_sub:
            self._node.destroy_subscription(self.map_sub)
        for p in self._paths.values():
            if p.sub:
                self._node.destroy_subscription(p.sub)
        for p in self._polygons.values():
            if p.sub:
                self._node.destroy_subscription(p.sub)
        super(NavView, self).close()

    def _update(self):
        if self._map_item:
            self._scene.removeItem(self._map_item)

        pixmap = QPixmap.fromImage(self._map)
        self._map_item = self._scene.addPixmap(pixmap)

        # Everything must be mirrored
        self._mirror(self._map_item)

        # Add drag and drop functionality
        self.add_dragdrop(self._map_item)

        self.centerOn(self._map_item)
        self.show()

    def _update_path(self, name):
        old_item = None
        if name in self._paths.keys():
            old_item = self._paths[name].item

        self._paths[name].item = self._scene.addPath(
            self._paths[name].path, pen=QPen(QColor(*self._paths[name].color))
        )

        if old_item:
            self._scene.removeItem(old_item)

    def _update_polygon(self, name):
        old_item = None
        if name in self._polygons.keys():
            old_item = self._polygons[name].item

        self._polygons[name].item = self._scene.addPolygon(
            self._polygons[name].path, pen=QPen(QColor(*self._polygons[name].color))
        )

        if old_item:
            self._scene.removeItem(old_item)

    def _mirror(self, item):
        """
        Mirror any QItem to have correct orientation
        :param item:
        :return:
        """
        item.setTransform(QTransform().scale(1, -1).translate(0, -self._map_height))

    def point_qt_to_map(self, point):
        """
        Convert point from Qt to map coordinates

        :param point: tuple or list
        :return: map point
        """
        # Mirror point over y axis
        x = point[0]
        y = self._map_height - point[1]

        # Orientation might need to be taken into account
        return [
            x * self._map_resolution + self._map_origin.position.x,
            y * self._map_resolution + self._map_origin.position.y,
        ]

    def point_map_to_qt(self, point):
        """
        Convert point from map to qt coordinates

        :param point: tuple or list
        :return: map point
        """
        # Orientation might need to be taken into account
        x = (point[0] - self._map_origin.position.x) / self._map_resolution
        y = (point[1] - self._map_origin.position.y) / self._map_resolution

        # Mirror point over y axis
        return [x, self._map_height - y]
