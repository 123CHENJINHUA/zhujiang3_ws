import rospy
from std_msgs.msg import String
import threading
import queue
import time
from playsound3 import playsound
from robot_msgs.srv import ui_get, ui_getRequest, ui_getResponse

class UiNode:

    def __init__(self):
        # 订阅 /speach
        self.speach_sub = rospy.Subscriber("/speach", String, self.speach_callback)
        # ui客户端
        self.ui_get_client = rospy.ServiceProxy('/UI_get', ui_get)

        # 音频播放相关
        self.voice_msgs_path = "/home/cjh/zhujiang_ws/src/ui_pkg/voice_msgs"

        self.sound = playsound(f'{self.voice_msgs_path}/{99}.wav', block=False)

        self.is_music = False  # 用于判断是否正在播放音乐

        # 创建音频播放线程
        self.audio_thread = threading.Thread(target=self._audio_worker, daemon=True)
        self.audio_thread.start()
        
        # 启动10秒后发送"1,1,1,1"
        self.start_timer()

    def start_timer(self):
        """启动定时器，10秒后发送任务列表"""
        timer = threading.Timer(10.0, self.send_initial_task)
        timer.daemon = True
        timer.start()
        rospy.loginfo("Timer started, will send task list in 10 seconds")

    def send_initial_task(self):
        """发送初始任务列表"""
        rospy.loginfo("Sending initial task list: 1,1,1,1")
        self.call_ui_get("1,1,1,1")

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