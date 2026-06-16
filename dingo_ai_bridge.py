#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

class AIBridge:
    def __init__(self):
        rospy.init_node("dingo_ai_bridge")

        self.enabled = rospy.get_param("~enabled", True)
        self.mode = "IDLE"
        self.last_cmd_t = rospy.Time.now()

        # 输出到 Dingo 真实速度话题(改成你 Dingo 的实际 topic)
        self.dingo_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=2)

        rospy.Subscriber("/ai_camera/cmd", Twist, self.on_cmd)
        rospy.Subscriber("/ai_camera/mode", String, self.on_mode)

        self.enable_pub = rospy.Publisher(
            "/ai_camera/enable", Bool, queue_size=1, latch=True
        )
        self.enable_pub.publish(Bool(data=self.enabled))

        rospy.Service  # 这里也可以做 ros service 控制 enable
        rospy.Timer(rospy.Duration(0.1), self.watchdog)

    def on_cmd(self, msg):
        if not self.enabled:
            return
        self.last_cmd_t = rospy.Time.now()
        self.dingo_pub.publish(msg)

    def on_mode(self, msg):
        self.mode = msg.data

    def watchdog(self, _):
        # 0.5s 没收到 AI 命令 -> 停车,防止 AI 节点崩了机器狗失控
        if (rospy.Time.now() - self.last_cmd_t).to_sec() > 0.5:
            self.dingo_pub.publish(Twist())

    def set_enabled(self, on):
        self.enabled = on
        self.enable_pub.publish(Bool(data=on))
        if not on:
            self.dingo_pub.publish(Twist())

if __name__ == "__main__":
    bridge = AIBridge()
    rospy.spin()
