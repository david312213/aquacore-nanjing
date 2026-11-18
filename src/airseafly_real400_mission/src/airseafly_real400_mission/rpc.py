"""Local Unix-domain request transport, no network listener."""
import json
import socket
import socketserver


class CommandNotSent(RuntimeError):
    """Connection/startup failed before any action request was submitted."""


class Reply(dict):
    """Complete an accepted lifecycle action after flushing its response."""

    def __init__(self, body, after_reply):
        super().__init__(body)
        self.after_reply = after_reply


def request(path, body, timeout=10.):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect(str(path))
        sock.sendall((json.dumps(body, ensure_ascii=False) + '\n').encode())
        with sock.makefile('rb') as stream:
            data = stream.readline(2 * 1024 * 1024)
        if not data:
            raise RuntimeError('任务管理器没有返回结果')
        return json.loads(data.decode())


class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            line = self.rfile.readline(2 * 1024 * 1024)
            result = self.server.dispatch(json.loads(line.decode()))
        except Exception as exc:
            result = dict(ok=False, error=str(exc))
        try:
            self.wfile.write((json.dumps(result, ensure_ascii=False, allow_nan=False) + '\n').encode())
            self.wfile.flush()
        finally:
            # An accepted shutdown must also complete if the client disconnects.
            # Never start it inside dispatch: rospy can close this connection
            # or terminate the daemon before the success reply reaches the CLI.
            if isinstance(result, Reply):
                result.after_reply()
