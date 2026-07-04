#!/usr/bin/env python3
"""Filter DOSOD PerceptionTargets by classes selected from the web console."""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from ai_msgs.msg import PerceptionTargets


class DosodFilterNode(Node):
    def __init__(self):
        super().__init__('dosod_filter_node')
        self.declare_parameter('input_topic', '/hobot_dnn_detection')
        self.declare_parameter('output_topic', '/hobot_dnn_detection_filtered')
        self.declare_parameter('selected_classes_topic', '/dosod/selected_classes')
        self.declare_parameter('default_classes', ['cup'])

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.selected_topic = self.get_parameter('selected_classes_topic').value
        self.selected = self._clean(self.get_parameter('default_classes').value)

        selection_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PerceptionTargets, self.output_topic, 10)
        self.create_subscription(PerceptionTargets, self.input_topic, self._on_targets, 10)
        self.create_subscription(String, self.selected_topic, self._on_selection, selection_qos)
        self.get_logger().info(
            f'DOSOD filter: {self.input_topic} -> {self.output_topic}, selected={sorted(self.selected)}')

    def _clean(self, values):
        cleaned = set()
        for item in values or []:
            name = str(item).strip().lower()
            if name:
                cleaned.add(name)
        return cleaned or {'cup'}

    def _on_selection(self, msg: String):
        try:
            data = json.loads(msg.data)
            if isinstance(data, str):
                values = [data]
            else:
                values = list(data)
        except Exception:
            values = [p.strip() for p in msg.data.split(',')]
        self.selected = self._clean(values)
        self.get_logger().info(f'Selected DOSOD classes: {sorted(self.selected)}')

    def _target_labels(self, target):
        labels = []
        if getattr(target, 'type', ''):
            labels.append(target.type)
        for roi in getattr(target, 'rois', []):
            if getattr(roi, 'type', ''):
                labels.append(roi.type)
        return {label.strip().lower() for label in labels if label.strip()}

    def _keep(self, target):
        labels = self._target_labels(target)
        return bool(labels & self.selected)

    def _on_targets(self, msg: PerceptionTargets):
        out = PerceptionTargets()
        out.header = msg.header
        out.fps = msg.fps
        out.perfs = msg.perfs
        out.targets = [target for target in msg.targets if self._keep(target)]
        out.disappeared_targets = [target for target in msg.disappeared_targets if self._keep(target)]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DosodFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
