#!/usr/bin/env python

import rospy
from pymongo import MongoClient
from mongo_msg_db import MessageDb
from mongo_msg_db_msgs.msg import Collection
from rapid_msgs.msg import StaticCloud, StaticCloudInfo
from rapid_msgs.srv import GetStaticCloud, GetStaticCloudResponse
from rapid_msgs.srv import ListStaticClouds, ListStaticCloudsResponse
from rapid_msgs.srv import SaveStaticCloud, SaveStaticCloudResponse
from rapid_msgs.srv import RemoveStaticCloud, RemoveStaticCloudResponse
from rospy_message_converter import json_message_converter as jmc
from sensor_msgs.msg import PointCloud2


class StaticCloudDb(object):
    def __init__(self, db):
        self._db = db

    def serve_get_cloud(self, req):
        matched_count, cloud = self._db.find_msg(req.collection, req.id)
        response = GetStaticCloudResponse()
        if matched_count == 0:
            response.error = 'StaticCloud was not found.'
        else:
            response.cloud = cloud
        return response

    def serve_list_clouds(self, req):
        response = ListStaticCloudsResponse()
        messages = self._db.list(req.collection)
        for message in messages:
            info = StaticCloudInfo()
            info.id = message.id
            cloud = jmc.convert_json_to_ros_message(message.msg_type, message.json)
            info.name = cloud.name
            response.clouds.append(info)
        return response

    def serve_remove_cloud(self, req):
        deleted_count = self._db.delete(req.collection, req.id)
        response = RemoveStaticCloudResponse()
        if deleted_count == 0:
            response.error = 'StaticCloud already not in collection.'
        return response

    def serve_save_cloud(self, req):
        response = SaveStaticCloudResponse()
        response.id = self._db.insert_msg(req.collection, req.cloud)
        return response


def main():
    rospy.init_node('static_cloud_db')
    mongo_client = MongoClient()
    mongo_db = MessageDb(mongo_client)
    db = StaticCloudDb(mongo_db)
    get = rospy.Service('get_static_cloud', GetStaticCloud, db.serve_get_cloud)
    list_clouds = rospy.Service('list_static_clouds', ListStaticClouds,
                                db.serve_list_clouds)
    remove = rospy.Service('remove_static_cloud', RemoveStaticCloud,
                           db.serve_remove_cloud)
    save = rospy.Service('save_static_cloud', SaveStaticCloud,
                         db.serve_save_cloud)
    rospy.spin()


if __name__ == '__main__':
    main()
