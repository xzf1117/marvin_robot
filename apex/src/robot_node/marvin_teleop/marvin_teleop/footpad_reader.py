#!/usr/bin/env python3
from __future__ import annotations

import threading
from typing import Set

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32
try:
	from pynput import keyboard
except ImportError as exc:  # pragma: no cover - runtime dependency
	raise SystemExit(
		"pynput is required to run this script. Install with: pip install pynput"
	) from exc


SHIFT_KEY = "shift_l"
FOOTPAD_KEYS = ("left", "middle", "right")
FOOTPAD_VALUES = {
	"left": 1,
	"middle": 2,
	"right": 3,
}


class FootpadKeyNode(Node):
	def __init__(self) -> None:
		super().__init__("footpad_key_reader")
		self._publisherL = self.create_publisher(Int32, "control/footkey", 10)
		self._pressed_keys: Set[str] = set()
		self._last_state: int | None = None
		self._lock = threading.Lock()
		self._timer = self.create_timer(0.05, self.publish_current_state)

	def publish_state(self, value: int) -> None:
		self._publisherL.publish(Int32(data=value))
		self._last_state = value

	def publish_current_state(self) -> None:
		with self._lock:
			value = _current_footpad_value(self._pressed_keys)
		self.publish_state(value)


def _normalize_key(key: keyboard.Key | keyboard.KeyCode) -> str | None:
	if isinstance(key, keyboard.KeyCode) and key.char:
		return key.char.lower()
	if isinstance(key, keyboard.Key):
		if key == keyboard.Key.shift_l:
			return "shift_l"
		if key == keyboard.Key.left:
			return "left"
		if key == keyboard.Key.up:
			return "middle"
		if key == keyboard.Key.right:
			return "right"
	return None


def _current_footpad_value(pressed_keys: Set[str]) -> int:
	if SHIFT_KEY not in pressed_keys:
		return 0
	for key in FOOTPAD_KEYS:
		if key in pressed_keys:
			return FOOTPAD_VALUES[key]
	return 0


def on_press(key: keyboard.Key | keyboard.KeyCode, node: FootpadKeyNode) -> None:
	normalized = _normalize_key(key)
	if normalized is None:
		return
	with node._lock:
		node._pressed_keys.add(normalized)
		value = _current_footpad_value(node._pressed_keys)
	node.publish_state(value)


def on_release(key: keyboard.Key | keyboard.KeyCode, node: FootpadKeyNode) -> None:
	normalized = _normalize_key(key)
	if normalized is None:
		return
	with node._lock:
		node._pressed_keys.discard(normalized)
		value = _current_footpad_value(node._pressed_keys)
	node.publish_state(value)


def main() -> None:
	rclpy.init()
	node = FootpadKeyNode()
	node.publish_state(0)

	with keyboard.Listener(
		on_press=lambda key: on_press(key, node),
		on_release=lambda key: on_release(key, node),
		suppress=False,
	) as listener:
		try:
			while rclpy.ok():
				rclpy.spin_once(node, timeout_sec=0.1)
		except (KeyboardInterrupt, ExternalShutdownException):
			pass
		finally:
			node.destroy_node()
			if rclpy.ok():
				rclpy.shutdown()
			listener.stop()


if __name__ == "__main__":
	main()
