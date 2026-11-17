"""Bounded exact-state receipt ledger for the independent actuator Guard.

This is an observational safety input, never an alternative certificate.
It records only actual received CarState sequence/stamp pairs; a missing
physics tick cannot be reconstructed or promoted into actuation authority.
Pure Python for offline fault-injection tests, no ROS dependencies.
"""
from collections import OrderedDict
from dataclasses import dataclass
from enum import Enum


class ReceiptStatus(Enum):
    INSERTED = 'INSERTED'
    GAP_OBSERVED = 'GAP_OBSERVED'
    SEQUENCE_REGRESSION = 'SEQUENCE_REGRESSION'
    STAMP_REGRESSION = 'STAMP_REGRESSION'


class BindingStatus(Enum):
    EXACT_LATEST = 'EXACT_LATEST'
    PENDING_ONE_FUTURE = 'PENDING_ONE_FUTURE'
    HISTORICAL_NO_AUTHORITY = 'HISTORICAL_NO_AUTHORITY'
    MISSING = 'MISSING'
    STAMP_MISMATCH = 'STAMP_MISMATCH'
    STALE_ROS = 'STALE_ROS'
    STALE_STEADY = 'STALE_STEADY'
    BAD_TIME = 'BAD_TIME'


@dataclass(frozen=True)
class StateReceipt:
    sequence: int
    stamp_ros_ns: int
    received_steady_ns: int


class StateTimeline:
    """Finite state receipt buffer; O(depth) at worst, depth fixed at init.

    `lookup` is not certification; only independent Guard may issue Applied.
    """
    def __init__(self, depth=16, max_age_ns=100_000_000):
        if not isinstance(depth, int) or depth < 2 or depth > 256:
            raise ValueError('INVALID_TIMELINE_DEPTH')
        if not isinstance(max_age_ns, int) or max_age_ns <= 0:
            raise ValueError('INVALID_TIMELINE_AGE')
        self.depth = depth
        self.max_age_ns = max_age_ns
        self._receipts = OrderedDict()
        self.last_sequence = None
        self.last_stamp_ros_ns = None
        self.missing_ranges = []
        self.gap_count = 0
        self.total_received = 0
        self.maximum_receive_gap_ns = 0
        self._last_receive_ns = None

    def append(self, receipt):
        if (self.last_sequence is not None and
                receipt.sequence <= self.last_sequence):
            return ReceiptStatus.SEQUENCE_REGRESSION
        if (self.last_stamp_ros_ns is not None and
                receipt.stamp_ros_ns <= self.last_stamp_ros_ns):
            return ReceiptStatus.STAMP_REGRESSION
        status = ReceiptStatus.INSERTED
        if self.last_sequence is not None and receipt.sequence > self.last_sequence + 1:
            status = ReceiptStatus.GAP_OBSERVED
            self.gap_count += 1
            self.missing_ranges.append((self.last_sequence + 1, receipt.sequence - 1))
            self.missing_ranges = self.missing_ranges[-32:]
        if self._last_receive_ns is not None and receipt.received_steady_ns >= self._last_receive_ns:
            gap = receipt.received_steady_ns - self._last_receive_ns
            self.maximum_receive_gap_ns = max(self.maximum_receive_gap_ns, gap)
        self._last_receive_ns = receipt.received_steady_ns
        self.last_sequence = receipt.sequence
        self.last_stamp_ros_ns = receipt.stamp_ros_ns
        self._receipts[receipt.sequence] = receipt
        while len(self._receipts) > self.depth:
            self._receipts.popitem(last=False)
        self.total_received += 1
        return status

    def exact(self, sequence, stamp_ros_ns):
        receipt = self._receipts.get(sequence)
        if receipt is None:
            return None
        return receipt if receipt.stamp_ros_ns == stamp_ros_ns else None

    def binding(self, sequence, stamp_ros_ns, now_ros_ns, now_steady_ns):
        if self.last_sequence is None:
            return BindingStatus.MISSING
        receipt = self._receipts.get(sequence)
        if receipt is None:
            if sequence == self.last_sequence + 1:
                return BindingStatus.PENDING_ONE_FUTURE
            return BindingStatus.MISSING
        if receipt.stamp_ros_ns != stamp_ros_ns:
            return BindingStatus.STAMP_MISMATCH
        if sequence != self.last_sequence:
            return BindingStatus.HISTORICAL_NO_AUTHORITY
        ros_delta = now_ros_ns - receipt.stamp_ros_ns
        steady_delta = now_steady_ns - receipt.received_steady_ns
        if ros_delta < 0 or steady_delta < 0:
            return BindingStatus.BAD_TIME
        if ros_delta > self.max_age_ns:
            return BindingStatus.STALE_ROS
        if steady_delta > self.max_age_ns:
            return BindingStatus.STALE_STEADY
        return BindingStatus.EXACT_LATEST
