#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""静态 Web 控制台文件服务（walle_web_console/web）。"""

import os
import rospy
import rospkg
from http.server import HTTPServer, SimpleHTTPRequestHandler
from threading import Thread


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        rospy.logdebug(format % args)


def main():
    rospy.init_node('walle_web_server', anonymous=True)
    port = rospy.get_param('~port', 8080)
    web_dir = rospy.get_param('~web_dir', '')
    if not web_dir:
        try:
            pkg_path = rospkg.RosPack().get_path('walle_web_console')
            web_dir = os.path.join(pkg_path, 'web')
        except rospkg.ResourceNotFound:
            web_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'web')
    if not os.path.isdir(web_dir):
        rospy.logfatal('Web 目录不存在: %s', web_dir)
        return

    os.chdir(web_dir)
    server = HTTPServer(('0.0.0.0', port), QuietHandler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    rospy.loginfo('Web 控制台: http://0.0.0.0:%d/', port)
    rospy.loginfo('  本地访问: http://localhost:%d/', port)
    rospy.spin()
    server.shutdown()


if __name__ == '__main__':
    main()
