import rclpy
from rclpy.node import Node
from hesai_ros_driver.msg import UdpFrame
from std_srvs.srv import Trigger
from functools import partial
import numpy as np
from collections import deque

class PacketSubscriber(Node): 
  def __init__(self):
    super().__init__('packet_xorer')
    self.original_subscriber = self.create_subscription(UdpFrame, '/lidar_packets', partial(self.listener_callback, boolean_index=0), 1000)
    self.decompressed_subscriber = self.create_subscription(UdpFrame, '/lidar_packets_decompressed', partial(self.listener_callback, boolean_index=1), 1000)
    self.original_msgs = deque()
    self.decompressed_msgs = deque()
    self.count = 0
    self.non1080sizedpackets = 0
    self.erroringmessages = []
    self.republisher = self.create_publisher(UdpFrame, '/lidar_packets', 1000)
    self.republish_srv = self.create_service(Trigger, 'republish_erroring', self.republish_callback)
    self.max_packet_index = 0
    self.min_packet_index = 8799
    self.seen_index = np.zeros(8800, dtype=bool)
    self.seen_indexes = set()
    
  def listener_callback(self, msg, boolean_index):
    # Save the message depending on the topic
    arr = np.asarray([element for packet in msg.packets for element in packet.data], dtype=np.uint8)
    if boolean_index == 0:
      self.original_msgs.append(arr)
      #print(f"Received original message with {len(msg.packets)} packets, total size: {len(arr)} bytes")
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
      diff_bit_indexes = np.flatnonzero(np.unpackbits(xor_result)) % (1100*8)
      
      # Expected bit differences from reserved bytes: 1 unencoded byte per point × 8 bits
      expected_threshold = int(len(original_msg) / 1080 * 256 * 8)
      comparison = "equal to" if diff_count == expected_threshold else ("below" if diff_count < expected_threshold else "above")
      if comparison == "above":
        self.count += 1
      print(f"Received both messages, after xoring detected {diff_count} bit differences which is {diff_count / (len(original_msg) * 8) * 100:.2f} percent in the {len(original_msg)*8}bits long message")
      #self.max_packet_index = np.max([np.max([diff_bit_indexes]), self.max_packet_index])
      #self.min_packet_index = np.min([np.min(diff_bit_indexes), self.min_packet_index])
      #self.seen_indexes=self.seen_indexes.union(set(diff_bit_indexes.tolist()))
      #self.seen_index[diff_bit_indexes] = [True] * len(diff_bit_indexes)
      #completenss = np.all(self.seen_index[self.min_packet_index:self.max_packet_index])
      #print(f"Differing bit indexes modulo 8800: {self.min_packet_index} to {self.max_packet_index}, completeness: {completenss}")

      #print(f"Seen indexes: {len(self.seen_indexes)} unique differing bit indexes, completeness: {np.all(self.seen_index[self.min_packet_index:self.max_packet_index])}")
      #print(self.seen_indexes)
      #print(f"Reserved bits: "+str(np.unpackbits(original_msg[4:6]))+" "+str(np.unpackbits(original_msg[8]))+" "+str(np.unpackbits(original_msg[1032:1032+11])))

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
