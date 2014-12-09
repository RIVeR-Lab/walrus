#!/usr/bin/env python

import os
import rospy
import rospkg
import threading
import tornado.ioloop as ioloop
import tornado.web as web

class MainHandler(web.RequestHandler):
    def get(self):
        self.render("/index.html")

def main():
    rospy.init_node('walrus_web_server', anonymous=True)
    rospack = rospkg.RosPack()

    web_dir = os.path.join(rospack.get_path('walrus_web'), 'web')

    application = web.Application([
        (r"/(.*)", web.StaticFileHandler, {"path": web_dir}),
        (r"/", MainHandler)
    ])
    application.listen(8000)
    server_thread = threading.Thread(target=ioloop.IOLoop.instance().start)

    server_thread.start()
    rospy.spin()
    ioloop.IOLoop.instance().stop()
    server_thread.join()


if __name__ == '__main__':
    main()


