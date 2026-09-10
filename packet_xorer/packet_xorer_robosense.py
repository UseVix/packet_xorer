import rclpy
from rclpy.node import Node
from rslidar_msg.msg import RslidarPacket
from std_srvs.srv import Trigger
from functools import partial
import numpy as np

class PacketSubscriber(Node): 
  def __init__(self):
    super().__init__('packet_xorer')
    self.original_subscriber = self.create_subscription(RslidarPacket, '/rslidar_packets', partial(self.listener_callback, boolean_index=0), 10)
    self.decompressed_subscriber = self.create_subscription(RslidarPacket, '/lidar_packets_decompressed', partial(self.listener_callback, boolean_index=1), 10)
    self.original_msg = np.array([], dtype=np.uint8)
    self.decompressed_msg = np.array([], dtype=np.uint8)
    self.received_msg = [False, False]
    self.count = 0
    self.non1080sizedpackets = 0
    self.erroringmessages = []
    self.republisher = self.create_publisher(RslidarPacket, '/lidar_packets', 10)
    self.republish_srv = self.create_service(Trigger, 'republish_erroring', self.republish_callback)
    
  def listener_callback(self, msg, boolean_index):
    # Save the message depending on the topic
    arr = np.asarray([element for packet in msg.packets for element in packet.data], dtype=np.uint8)
    if boolean_index == 0:
      self.original_msg = np.concatenate((self.original_msg, arr))
      """
      for packet in msg.packets:
        if len(packet.data) != 1080:
          print(f"Packet with size {len(packet.data)} received")
          self.erroringmessages.append(msg)
          self.non1080sizedpackets += 1
      print(f"Total non-1080 sized packets: {self.non1080sizedpackets}")
      """

      
    else:
      self.decompressed_msg = np.concatenate((self.decompressed_msg, arr))
      
    self.received_msg[boolean_index] = True
    
    # When both have been received, do the bitwise XOR comparison
    if self.received_msg[0] and self.received_msg[1] and len(self.original_msg) == len(self.decompressed_msg):
      # Vectorized XOR
      xor_result = np.bitwise_xor(self.original_msg, self.decompressed_msg)
      
      # Vectorized bit-counting (unpackbits turns bytes into an array of 0s and 1s, which we can just sum)
      diff_count = np.sum(np.unpackbits(xor_result))
      print(f"Reserved bits: "+str(np.unpackbits(self.original_msg[-4:])))
      # Expected bit differences from reserved bytes: 1 unencoded byte per point × 8 bits
      #expected_threshold = int(len(self.original_msg) / 1080 * 256 * 8)
      #comparison = "equal to" if diff_count == expected_threshold else ("below" if diff_count < expected_threshold else "above")
      #if comparison == "above":
      #  self.count += 1
      print(f"Received both messages, after xoring detected {diff_count} bit differences")
      print()
      #print(f"Total incorrect packets: {self.count}")
      self.original_msg = np.array([], dtype=np.uint8)
      self.decompressed_msg = np.array([], dtype=np.uint8)
      self.received_msg=[False,False]

  def republish_callback(self, request, response):
    count = len(self.erroringmessages)
    if count == 0:
      response.success = False
      response.message = 'No erroring messages stored'
      print('No erroring messages to republish')
    else:
      for msg in self.erroringmessages:
        self.republisher.publish(msg)
      response.success = True
      response.message = f'Republished {count} erroring messages'
      print(f'Republished {count} erroring messages on /lidar_packets_replay')
    return response
      
def main():
    print('Hi from packet_xorer.')
    rclpy.init()
    node = PacketSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
