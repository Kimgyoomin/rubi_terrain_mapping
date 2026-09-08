"""Bounded, non-blocking input queue for timestamped TF registration.

This module deliberately has no ROS or CUDA imports so that the queue's ordering
and resource policies can be regression-tested in a portable environment.
"""

from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Optional, Tuple


@dataclass(frozen=True)
class PendingPointCloud:
    message: Any
    subscriber_key: str
    stamp_ns: int
    received_monotonic: float
    payload_bytes: int


class PendingPointCloudQueue:
    """FIFO with strict stamp ordering and count/payload bounds.

    New inputs must have a positive, strictly increasing stamp. When adding an
    otherwise valid input would exceed a bound, oldest queued inputs are
    discarded first. A single payload larger than the byte budget is rejected.
    """

    def __init__(self, max_scans: int, max_bytes: int) -> None:
        if max_scans <= 0:
            raise ValueError("tf_queue_size must be positive")
        if max_bytes <= 0:
            raise ValueError("tf_queue_max_bytes must be positive")
        self.max_scans = max_scans
        self.max_bytes = max_bytes
        self._items: Deque[PendingPointCloud] = deque()
        self._payload_bytes = 0
        self._last_received_stamp_ns: Optional[int] = None

    def __len__(self) -> int:
        return len(self._items)

    @property
    def payload_bytes(self) -> int:
        return self._payload_bytes

    @property
    def last_received_stamp_ns(self) -> Optional[int]:
        return self._last_received_stamp_ns

    def peek(self) -> Optional[PendingPointCloud]:
        return self._items[0] if self._items else None

    def pop(self) -> PendingPointCloud:
        item = self._items.popleft()
        self._payload_bytes -= item.payload_bytes
        return item

    def clear(self) -> int:
        count = len(self._items)
        self._items.clear()
        self._payload_bytes = 0
        return count

    def push(self, item: PendingPointCloud) -> Tuple[str, Tuple[PendingPointCloud, ...]]:
        """Return (result, overflow_drops).

        result is one of accepted, duplicate, out_of_order, zero_stamp, or
        oversize. Overflow drops are ordered oldest-first.
        """
        if item.stamp_ns <= 0:
            return "zero_stamp", ()
        if item.payload_bytes < 0:
            raise ValueError("payload_bytes cannot be negative")
        if item.payload_bytes > self.max_bytes:
            return "oversize", ()
        if self._last_received_stamp_ns is not None:
            if item.stamp_ns == self._last_received_stamp_ns:
                return "duplicate", ()
            if item.stamp_ns < self._last_received_stamp_ns:
                return "out_of_order", ()

        # Remember accepted input stamps even if resource pressure later evicts
        # them, so retransmission cannot fuse a scan twice or travel backwards.
        self._last_received_stamp_ns = item.stamp_ns
        self._items.append(item)
        self._payload_bytes += item.payload_bytes

        dropped = []
        while len(self._items) > self.max_scans or self._payload_bytes > self.max_bytes:
            dropped.append(self.pop())
        return "accepted", tuple(dropped)
