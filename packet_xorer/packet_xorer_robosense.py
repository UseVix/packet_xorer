import rclpy
from rclpy.node import Node
from rslidar_msg.msg import RslidarPacket
from std_srvs.srv import Trigger
from functools import partial
import numpy as np
from collections import deque

class PacketSubscriber(Node): 
  def __init__(self):
    super().__init__('packet_xorer')
    self.original_subscriber = self.create_subscription(RslidarPacket, '/lidar_packets', partial(self.listener_callback, boolean_index=0), 1000)
    self.decompressed_subscriber = self.create_subscription(RslidarPacket, '/lidar_packets_decompressed', partial(self.listener_callback, boolean_index=1), 1000)
    self.original_msgs = deque()
    self.decompressed_msgs = deque()
    self.count = 0
    self.non1080sizedpackets = 0
    self.erroringmessages = []
    self.republisher = self.create_publisher(RslidarPacket, '/lidar_packets', 10)
    self.republish_srv = self.create_service(Trigger, 'republish_erroring', self.republish_callback)
    self.MSOP_counter = 0
    self.DIFOP_counter = 0
    self.unknown_counter = 0
    
  def listener_callback(self, msg, boolean_index):
    # Save the message depending on the topic
    arr = np.asarray([element for element in msg.data], dtype=np.uint8)
    if boolean_index == 0:
      if np.array_equal(arr[:4], [0x55, 0xAA, 0x05, 0x5A]):
        self.original_msgs.append(arr)
        self.MSOP_counter += 1
      elif np.array_equal(arr[:8], [0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55]):
        self.DIFOP_counter += 1
        print("DIFOP")
        print("Counters: MSOP: {}, DIFOP: {}, Unknown: {}".format(self.MSOP_counter, self.DIFOP_counter, self.unknown_counter))
      else:
        self.unknown_counter += 1
        print("Unknown packet type")
        print("Counters: MSOP: {}, DIFOP: {}, Unknown: {}".format(self.MSOP_counter, self.DIFOP_counter, self.unknown_counter))
        print(arr[:8].tolist())
      """
      for packet in msg.packets:
        if len(packet.data) != 1080:
          print(f"Packet with size {len(packet.data)} received")
          self.erroringmessages.append(msg)
          self.non1080sizedpackets += 1
      print(f"Total non-1080 sized packets: {self.non1080sizedpackets}")
      """

      
    else:
      self.decompressed_msgs.append(arr)
      
    
    # When both have been received, do the bitwise XOR comparison
    if self.original_msgs and self.decompressed_msgs:
      original_msg = self.original_msgs.popleft()
      decompressed_msg = self.decompressed_msgs.popleft()

      if len(original_msg) != len(decompressed_msg):
        print(f"Cannot XOR messages with different sizes: {len(original_msg)} and {len(decompressed_msg)}")
        return

      # Vectorized XOR
      xor_result = np.bitwise_xor(original_msg, decompressed_msg)
      
      # Vectorized bit-counting (unpackbits turns bytes into an array of 0s and 1s, which we can just sum)
      diff_count = np.sum(np.unpackbits(xor_result))
      #print(f"Reserved bits: "+str(np.unpackbits(original_msg[-4:])))
      # Expected bit differences from reserved bytes: 1 unencoded byte per point × 8 bits
      #expected_threshold = int(len(self.original_msg) / 1080 * 256 * 8)
      #comparison = "equal to" if diff_count == expected_threshold else ("below" if diff_count < expected_threshold else "above")
      #if comparison == "above":
      #  self.count += 1
      print(f"Received both messages, after xoring detected {diff_count} bit differences in {len(original_msg)*8} bit long messages that is {diff_count/(len(original_msg)*8)*100:.2f}% different")
      
      #print(f"Total incorrect packets: {self.count}")

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
