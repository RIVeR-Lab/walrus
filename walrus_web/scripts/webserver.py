#!/usr/bin/env python

import os
import rospy
import rospkg
import threading
import tornado.ioloop as ioloop
import tornado.web as web


def main():
    rospy.init_node('walrus_web_server')
    rospack = rospkg.RosPack()

    web_dir = os.path.join(rospack.get_path('walrus_web'), 'web')

    application = web.Application([
        (r"/()", web.StaticFileHandler, {'path': os.path.join(web_dir, 'index.html')}),
        (r"/(.*)", web.StaticFileHandler, {"path": web_dir})
    ], debug=True)
    application.listen(rospy.get_param("~port", 8080))
    server_thread = threading.Thread(target=ioloop.IOLoop.instance().start)

    server_thread.start()
    rospy.spin()
    ioloop.IOLoop.instance().stop()
    server_thread.join()


if __name__ == '__main__':
    main()


