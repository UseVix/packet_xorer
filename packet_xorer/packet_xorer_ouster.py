import rclpy
from rclpy.node import Node
from ouster_sensor_msgs.msg import PacketMsg
from std_srvs.srv import Trigger
from functools import partial
import numpy as np
from collections import deque
import numpy as np

def print_reserved_bits(packet_data):
    # Define the little-endian structure of the 24,896-byte OS-128 packet
    channel_block_dtype = np.dtype([
        ('word0', '<u4'),
        ('word1', '<u4'),
        ('word2', '<u4')
    ])

    measurement_block_dtype = np.dtype([
        ('header', '<u4', 4),
        ('channels', channel_block_dtype, 128),
        ('status', '<u4', 1)
    ])

    packet_dtype = np.dtype([
        ('blocks', measurement_block_dtype, 16)
    ])

    # Parse the packet
    packet = np.frombuffer(packet_data, dtype=packet_dtype)
    blocks = packet['blocks'][0]
    
    # Extract and print the reserved bits
    for block_idx in range(16):
        print(f"\n--- Measurement Block {block_idx} ---")
        
        # Header Reserved Bits (Word 3, bits 24-31)
        header_word3 = blocks['header'][block_idx, 3]
        header_res = (header_word3 >> 24) & 0xFF
        print(f"Header Reserved: {header_res:08b}")
        
        # Extract arrays for all 128 channels in this block
        w0 = blocks['channels']['word0'][block_idx]
        w1 = blocks['channels']['word1'][block_idx]
        w2 = blocks['channels']['word2'][block_idx]
        
        # Mask out the reserved bits for all channels simultaneously
        ch_res_20_27 = (w0 >> 20) & 0xFF
        ch_res_29_31 = (w0 >> 29) & 0x07
        ch_res_40_47 = (w1 >> 8) & 0xFF   
        ch_res_80_95 = (w2 >> 16) & 0xFFFF 
        
        # Print the channels up to range(16)
        for ch_idx in range(16): 
            print(f"  Channel {ch_idx}:")
            print(f"    Bits 20-27: {ch_res_20_27[ch_idx]:08b}")
            print(f"    Bits 29-31: {ch_res_29_31[ch_idx]:03b}")
            print(f"    Bits 40-47: {ch_res_40_47[ch_idx]:08b}")
            print(f"    Bits 80-95: {ch_res_80_95[ch_idx]:016b}")

# Example usage:
# print_reserved_bits(original_msg)
class PacketSubscriber(Node): 
  def __init__(self):
    super().__init__('packet_xorer')
    self.original_subscriber = self.create_subscription(PacketMsg, '/lidar_packets', partial(self.listener_callback, boolean_index=0), 10)
    self.decompressed_subscriber = self.create_subscription(PacketMsg, '/lidar_packets_decompressed', partial(self.listener_callback, boolean_index=1), 10)
    self.original_msgs = deque()
    self.decompressed_msgs = deque()
    self.count = 0
    self.non1080sizedpackets = 0
    self.erroringmessages = []
    self.republisher = self.create_publisher(PacketMsg, '/lidar_packets', 10)
    self.republish_srv = self.create_service(Trigger, 'republish_erroring', self.republish_callback)
    
  def listener_callback(self, msg, boolean_index):
    # Save the message depending on the topic
    arr = np.asarray([element for element in msg.buf], dtype=np.uint8)
    if boolean_index == 0:
      self.original_msgs.append(arr[:-1])  # Exclude the last byte for original messages
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
      
      # Expected bit differences from reserved bytes: 1 unencoded byte per point × 8 bits
      expected_threshold = 4488
      comparison = "equal to" if diff_count == expected_threshold else ("below" if diff_count < expected_threshold else "above")
      if comparison == "above":
        self.count += 1
      print(f"Received both messages, after xoring detected {diff_count} bit differences")
      #print(f"Reserved bits: "+str(np.unpackbits(original_msg)[120:128])+" "+str(np.unpackbits(original_msg)[128+80:128+96]))
      #print_reserved_bits(original_msg)
      print(f"Total incorrect packets: {self.count}")

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
