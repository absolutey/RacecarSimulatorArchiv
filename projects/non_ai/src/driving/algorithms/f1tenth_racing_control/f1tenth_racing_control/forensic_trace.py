"""Bounded async *diagnostics only*. Never grants actuator authority.

Emitting ROS callbacks must not synchronously open/write/close a file on each
state, watchdog, intent or safety verdict. Trace loss is explicitly counted;
control and fail-closed decisions never consult this writer.
"""
import atexit
import json
import os
import queue
import threading
import time
from pathlib import Path

_PATH = os.environ.get('F1TENTH_FORENSIC_TRACE_DIR')
_QUEUE = queue.Queue(maxsize=16384) if _PATH else None
_SHUTDOWN = object()
_DROPPED = 0


def _writer():
    global _DROPPED
    root = Path(_PATH)
    try:
        root.mkdir(parents=True, exist_ok=True)
        path = root / ('%s_%d.jsonl' % ('GUARD' if 'actuator_guard' in os.environ.get(
            'F1TENTH_DIAGNOSTIC_ROLE', '') else 'TRACE', os.getpid()))
        # Each event carries its role; separate the two roles in two files.
        handles = {}
        while True:
            packet = _QUEUE.get()
            if packet is _SHUTDOWN:
                break
            try:
                role, data = packet
                if role not in handles:
                    handles[role] = open(root / ('%s_%d.jsonl' % (role, os.getpid())),
                                         'a', encoding='utf-8', buffering=65536)
                # JSON encoding belongs to the diagnostics writer, never to
                # a time-constrained Guard/R-B callback. The queue carries
                # primitive-valued snapshots captured at enqueue time.
                handles[role].write(json.dumps(
                    data,sort_keys=True,separators=(',', ':'))+'\n')
            except (OSError, ValueError, TypeError):
                _DROPPED += 1
        for handle in handles.values():
            handle.flush()
            handle.close()
    except (OSError, ValueError):
        pass


def _close():
    if _QUEUE is not None:
        try:
            _QUEUE.put_nowait(_SHUTDOWN)
        except queue.Full:
            pass
        _THREAD.join(timeout=0.5)


if _QUEUE is not None:
    _THREAD = threading.Thread(target=_writer, name='f1tenth-trace-writer', daemon=True)
    _THREAD.start()
    atexit.register(_close)


def _emit(role, event, **fields):
    global _DROPPED
    if _QUEUE is None:
        return
    try:
        data = {'role': role, 'event': event, 'wall_ns': time.time_ns(),
                'monotonic_ns': time.monotonic_ns(), 'pid': os.getpid(), **fields}
        # Fail-closed actuation never reads this queue. Encoding in the
        # writer avoids holding the GIL for JSON conversion on every Guard
        # certificate, R-B physics receipt and scheduler callback.
        _QUEUE.put_nowait((role, data))
    except queue.Full:
        _DROPPED += 1


def trace_rb(event, **fields):
    _emit('RB', event, **fields)


def trace_guard(event, **fields):
    _emit('GUARD', event, **fields)
