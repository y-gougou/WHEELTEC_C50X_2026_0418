#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tiny static file server for the web teleop dashboard.
"""

import os
import threading

import rospy

try:
    from http.server import SimpleHTTPRequestHandler
except ImportError:
    from SimpleHTTPServer import SimpleHTTPRequestHandler

try:
    from socketserver import TCPServer
except ImportError:
    from SocketServer import TCPServer


class ReusableTCPServer(TCPServer):
    allow_reuse_address = True


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, _format, *_args):
        return


class StaticDashboardServer(object):
    def __init__(self):
        self.host = rospy.get_param("~host", "0.0.0.0")
        self.port = int(rospy.get_param("~port", 8000))
        self.package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.web_root = os.path.join(self.package_root, "web")

        if not os.path.isdir(self.web_root):
            raise RuntimeError("Web root does not exist: %s" % self.web_root)

        self.server = None
        self.server_thread = None

    def start(self):
        os.chdir(self.web_root)
        self.server = ReusableTCPServer((self.host, self.port), QuietHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "web dashboard server started at http://%s:%d (serving %s)",
            self.host,
            self.port,
            self.web_root,
        )

    def shutdown(self):
        if self.server is not None:
            try:
                self.server.shutdown()
                self.server.server_close()
            except Exception:
                pass
            self.server = None


if __name__ == "__main__":
    rospy.init_node("web_dashboard_server", anonymous=False)
    server = StaticDashboardServer()
    server.start()
    rospy.spin()
