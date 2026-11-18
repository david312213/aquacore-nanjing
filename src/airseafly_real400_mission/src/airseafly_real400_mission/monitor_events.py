"""Read-only terminal event recovery; never imported by the flight manager.

RPC history, ROS-style batches and the append-only journal share boot/sequence
identities. Reading the journal here does not add work to the control callback.
"""
import json
import os
from pathlib import Path
import threading


def packet_events(packet):
    if not isinstance(packet, dict):
        return []
    items = []
    for key in ('events', 'recent_events'):
        if isinstance(packet.get(key), list):
            items.extend(item for item in packet[key] if isinstance(item, dict))
    if 'seq' in packet and 'event' in packet:
        items.append({key: value for key, value in packet.items()
                      if key not in ('events', 'recent_events')})
    return items


class EventMonitor:
    def __init__(self, after=0, emit=lambda event: None, warn=lambda message: None):
        self.lock = threading.RLock()
        self.cursor = int(after)
        self.rpc_cursor = self.cursor
        self.boot = None
        self.emit, self.warn = emit, warn
        self.seen = set()
        self.highwater = self.cursor
        self.path = None
        self.offset = 0
        self.file_identity = None
        self.receipts = []
        self.warned = set()
        self.gap_start = None

    def _boot(self, value):
        if not value or value == self.boot:
            return
        if self.boot is not None:
            self.cursor = self.highwater = 0
            self.rpc_cursor = 0
            self.seen.clear()
            self.offset = 0
            self.file_identity = None
            self.receipts = []
            self.warned.clear()
            self.gap_start = None
        self.boot = value

    def after(self):
        with self.lock:
            return self.cursor

    def poll_after(self):
        # Transport progress is independent of journal recovery. Otherwise a
        # permanent gap would repeatedly fetch the same full cache, defeating
        # the RPC's existing long poll and loading the manager unnecessarily.
        with self.lock:
            return self.rpc_cursor

    def update_status(self, status):
        with self.lock:
            self._boot(status.get('boot_id'))
            directory = status.get('log_dir')
            path = Path(directory)/'events.jsonl' if directory else None
            if path != self.path:
                self.path, self.offset, self.file_identity = path, 0, None
            self.highwater = max(self.highwater, int(status.get('last_event_seq') or 0))
            self.receipts = list(status.get('payload_records') or [])

    def _warning(self, key, message):
        if key not in self.warned:
            self.warned.add(key)
            self.warn(message)

    def _journal(self):
        if self.path is None:
            return []
        rows = []
        try:
            with self.path.open('rb') as stream:
                stat = os.fstat(stream.fileno())
                identity = (stat.st_dev, stat.st_ino)
                if identity != self.file_identity or stat.st_size < self.offset:
                    self.offset = 0
                    self.file_identity = identity
                stream.seek(self.offset)
                # Snapshot EOF so a busy writer cannot keep this read running
                # indefinitely. A partial line is read again on the next poll.
                while stream.tell() < stat.st_size:
                    start = stream.tell()
                    line = stream.readline(stat.st_size-start)
                    if not line.endswith(b'\n'):
                        break
                    self.offset = stream.tell()
                    try:
                        item = json.loads(line)
                    except (ValueError, UnicodeError):
                        self._warning(('line', str(self.path), start),
                                      '日志有一条无法解析的记录，保留原文件并继续显示可读取的事件。')
                        continue
                    if isinstance(item, dict) and 'seq' in item:
                        rows.append(item)
        except OSError as exc:
            self._warning(('file', str(self.path), str(exc)),
                          '暂时无法读取完整事件日志；继续显示缓存事件和投放回执：'+str(exc))
        return rows

    def consume(self, packet=None):
        with self.lock:
            journal = self._journal()
            rows = []
            if packet:
                packet_boot = packet.get('boot_id')
                if self.boot is None:
                    self._boot(packet_boot)
                # The status selects the current boot. A late response from
                # a previous daemon must not switch the terminal back to it.
                if not packet_boot or packet_boot == self.boot:
                    rows.extend(packet_events(packet))
                    self.highwater = max(self.highwater, int(packet.get('last_seq') or 0))
                    self.rpc_cursor = max(self.rpc_cursor, int(packet.get('last_seq') or 0))
            rows.extend(self.receipts)
            # The journal retains the full state; RPC/ROS state_changed items
            # deliberately contain a compact output. Prefer the full record.
            rows.extend(journal)
            by_seq = {}
            for item in rows:
                if item.get('boot_id') != self.boot:
                    continue
                seq = item.get('seq')
                if isinstance(seq, int) and not isinstance(seq, bool) and seq > self.cursor:
                    by_seq[seq] = item
            for seq in sorted(by_seq):
                if seq not in self.seen:
                    self.emit(by_seq[seq])
                    self.seen.add(seq)
                    self.highwater = max(self.highwater, seq)
            # Never skip a missing sequence just because a packet advertises
            # a later last_seq. Delayed journal lines can still fill that gap.
            while self.cursor+1 in self.seen:
                self.cursor += 1
                self.seen.remove(self.cursor)
            if self.cursor < self.highwater:
                if self.gap_start != self.cursor+1:
                    self.gap_start = self.cursor+1
                    self.warn('事件 #%d 起尚有缺口，继续从原始日志补齐；已收到的回执照常显示。' % self.gap_start)
            else:
                self.gap_start = None
