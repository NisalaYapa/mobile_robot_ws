from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.transport13 import Node
 
import time
 
def main():
    node = Node()
    stringmsg_topic = "/example_stringmsg_topic"
    vector3d_topic = "/example_vector3d_topic"
    pub_stringmsg = node.advertise(stringmsg_topic, StringMsg)
    pub_vector3d = node.advertise(vector3d_topic, Vector3d)
 
    vector3d_msg = Vector3d()
    vector3d_msg.x = 10
    vector3d_msg.y = 15
    vector3d_msg.z = 20
 
    stringmsg_msg = StringMsg()
    stringmsg_msg.data = "Hello" 
    try:
        count = 0
        while True:
          count += 1
          vector3d_msg.x = count
          if not pub_stringmsg.publish(stringmsg_msg):
              break
          print("Publishing 'Hello' on topic [{}]".format(stringmsg_topic))
          if not pub_vector3d.publish(vector3d_msg):
              break
          print("Publishing a Vector3d on topic [{}]".format(vector3d_topic))
          time.sleep(0.1)
 
    except KeyboardInterrupt:
        pass