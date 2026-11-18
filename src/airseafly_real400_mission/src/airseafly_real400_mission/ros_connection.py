"""Read-only ROS graph checks for reusing persistent local processes."""
import xmlrpc.client


class BoundedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.5
        return connection


def process_registered(master_uri, pid, node_prefix, subscriptions=()):
    """A responsive local socket alone does not survive a ROS master restart.

    Match both graph registration and the serving PID. For the mission manager,
    also require its position/controller subscriptions to belong to this graph.
    This is called before submitting a new task, never from a flight loop.
    """
    if type(pid) is not int or pid <= 0:
        return False
    caller = '/airseafly_connection_check'
    try:
        with xmlrpc.client.ServerProxy(master_uri, transport=BoundedTransport()) as master:
            code, message, graph = master.getSystemState(caller)
            if code != 1:
                raise RuntimeError(message)
            registered = {name for section in graph for _, names in section for name in names}
            candidates = sorted(name for name in registered
                                if name == node_prefix or name.startswith(node_prefix + '_'))
            subscriber_map = dict(graph[1])
            for name in candidates:
                if any(name not in subscriber_map.get(topic, []) for topic in subscriptions):
                    continue
                code, _, uri = master.lookupNode(caller, name)
                if code != 1:
                    continue
                try:
                    with xmlrpc.client.ServerProxy(uri, transport=BoundedTransport()) as node:
                        code, _, actual_pid = node.getPid(caller)
                    if code == 1 and actual_pid == pid:
                        return True
                except (OSError, ValueError, xmlrpc.client.Error):
                    continue
    except (OSError, ValueError, RuntimeError, xmlrpc.client.Error) as exc:
        raise RuntimeError('无法确认当前ROS连接：%s' % exc) from exc
    return False
