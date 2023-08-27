import rclpy
import rclpy.qos
from rclpy.node import Node
import sys
import socket
import threading


class UdpMulticastBeacon(Node):
  def __init__(self):
    super().__init__("udp_multicast_beacon")

    self.declare_parameter("multicast_group_addr", "224.1.1.1")
    self.declare_parameter("multicast_port", 5007)
    # self.declare_parameter("multicast_port", 50000)
    self.declare_parameter("multicast_listen_port", 5008)
    self.declare_parameter("reply_port", 5009)
    self.declare_parameter("multicast_ttl", 2)
    self.declare_parameter("period_sec", 1.0)
    self.declare_parameter("name", "robot")

    # regarding socket.IP_MULTICAST_TTL
    # ---------------------------------
    # for all packets sent, after two hops on the network the packet will not 
    # be re-sent/broadcast (see https://www.tldp.org/HOWTO/Multicast-HOWTO-6.html)

    # For Python 3, change next line to 'sock.sendto(b"robot", ...' to avoid the
    # "bytes-like object is required" msg (https://stackoverflow.com/a/42612820)

    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, self.get_parameter("multicast_ttl").value)

    # print(socket.INADDR_ANY)
    mreq = socket.inet_aton(self.get_parameter("multicast_group_addr").value) + socket.inet_aton("0.0.0.0")
    self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    self.sock.bind(("0.0.0.0", self.get_parameter("multicast_listen_port").value))

    self.timer = self.create_timer(self.get_parameter("period_sec").value, self.timer_callback)
    self.listen_thread = threading.Thread(target=self.listen, daemon=True)
    self.listen_thread.start()


  def timer_callback(self):
    try:
      self.sock.sendto(b"robot_discovery_service:" + self.get_parameter("name").value.encode("utf-8"),
        (self.get_parameter("multicast_group_addr").value,
        self.get_parameter("multicast_port").value))
    except OSError as e:
      print(e)
  
  def listen(self):
    while True:
      data, addr = self.sock.recvfrom(1000)
      s = data.decode("utf-8")
      print("multicast received")
      if s == f"robot_discovery_service:{self.get_parameter('name').value}":
        reply_dest = (addr[0], self.get_parameter("reply_port").value)
        self.get_logger().info(f"sending reply to {reply_dest}")
        self.sock.sendto(b"", reply_dest)


def main(args=None) -> None:
  rclpy.init(args=sys.argv)
  node = UdpMulticastBeacon()
  try:
      rclpy.spin(node)
  except KeyboardInterrupt:
      pass
  finally:
      node.destroy_node()
      rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)

