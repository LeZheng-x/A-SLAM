import rospy
import time
from keyBoard_Control.msg import control
from pynput import keyboard#首先导入模块
pub = rospy.Publisher('keyBoard_Control', control, queue_size=10)

def key_press(key):#定义按键按下时触发的函数
    try:
        hello_str = control()
        if key.char =='q':
            hello_str.q_ = True
        elif key.char =='w':
            hello_str.w_ = True
        elif key.char =='e':
            hello_str.e_ = True
        elif key.char =='a':
            hello_str.a_ = True
        elif key.char =='s':
            hello_str.s_ = True
        elif key.char =='d':
            hello_str.d_ = True
        elif key.char =='p':
            hello_str.p_ = True
        pub.publish(hello_str)

    except AttributeError:     
        pass

def on_release(key):
    if key == keyboard.Key.esc:
        return False

if __name__ == '__main__':
    try:
        rospy.init_node('controller', anonymous=True)

        with keyboard.Listener(on_press=key_press,on_release=on_release) as listener:
            listener.join()
    except rospy.ROSInterruptException:
        pass
