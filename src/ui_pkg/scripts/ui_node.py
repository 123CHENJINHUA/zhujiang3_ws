import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import threading
import queue
import time
from playsound3 import playsound
from robot_msgs.srv import ui_get, ui_getRequest, ui_getResponse

class UiNode:

    def __init__(self):
        # 订阅 /speach
        self.speach_sub = rospy.Subscriber("/speach", String, self.speach_callback)
        # 订阅 /joy 手柄消息
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        # ui客户端
        self.ui_get_client = rospy.ServiceProxy('/UI_get', ui_get)

        # 音频播放相关
        self.voice_msgs_path = "/home/cjh/zhujiang_ws/src/ui_pkg/voice_msgs"

        self.sound = playsound(f'{self.voice_msgs_path}/{99}.wav', block=False)

        self.is_music = False  # 用于判断是否正在播放音乐

        # 手柄按键状态跟踪
        self.prev_a_button = 0  # 记录上一次A键状态，用于检测按键按下事件
        self.prev_b_button = 0  # 记录上一次B键状态
        self.prev_y_button = 0  # 记录上一次Y键状态
        self.prev_x_button = 0  # 记录上一次X键状态

        # 创建音频播放线程
        self.audio_thread = threading.Thread(target=self._audio_worker, daemon=True)
        self.audio_thread.start()

    def joy_callback(self, msg):
        """处理手柄消息"""
        try:
            # PS2手柄按键映射（常见配置）：
            # A键：索引0, B键：索引1, X键：索引2, Y键：索引3
            if len(msg.buttons) >= 4:
                current_a_button = msg.buttons[0]
                current_b_button = msg.buttons[1]
                current_x_button = msg.buttons[3]
                
                # 检测A键按下事件（从0变为1）
                if current_a_button == 1 and self.prev_a_button == 0:
                    rospy.loginfo("A button pressed, sending task list: 1,1,1,1")
                    self.call_ui_get("1,1,1,1")
                
                # 检测B键按下事件
                if current_b_button == 1 and self.prev_b_button == 0:
                    rospy.loginfo("B button pressed, sending task list: 1,1,1,2")
                    self.call_ui_get("1,1,1,2")
                
                # 检测X键按下事件
                if current_x_button == 1 and self.prev_x_button == 0:
                    rospy.loginfo("X button pressed, sending task list: 1,1,1,3")
                    self.call_ui_get("1,1,1,3")
                
                
                # 更新按键状态
                self.prev_a_button = current_a_button
                self.prev_b_button = current_b_button
                self.prev_x_button = current_x_button
            
        except Exception as e:
            rospy.logerr(f"Joy callback error: {e}")

    def _audio_worker(self):
        while not rospy.is_shutdown():
            try:
                if self.is_music and not self.sound.is_alive():  # 检查音频路径是否有效且当前没有音频在播放
                    self.sound = playsound(f'{self.voice_msgs_path}/{100}.wav', block=False)  # 播放音频
            except Exception as e:
                rospy.logerr(f"Audio playback error: {e}")

    def speach_callback(self, msg):
        wav_path = f'{self.voice_msgs_path}/{msg.data}.wav'
        rospy.loginfo("Received speech message: %s", msg.data)
        if msg.data == '100':
            self.is_music = True  # 设置正在播放音乐状态
        else:
            if self.is_music:
                self.sound.stop()  # 停止当前音乐
                self.is_music = False  # 重置音乐状态
            playsound(wav_path, block=True)  # 播放语音消息
            

    # 客户端
    def call_ui_get(self, delivery_list):
        try:
            req = ui_getRequest()
            req.delivery_list = delivery_list
            resp = self.ui_get_client.call(req)
            if resp.received:
                rospy.loginfo("UI_get call success, task list received.")
            else:
                rospy.logwarn("UI_get call failed, task list not accepted.")
            return resp
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None


def main():
    rospy.init_node('ui_node')
    ui_node = UiNode()
    rospy.spin()

if __name__ == '__main__':
    main()