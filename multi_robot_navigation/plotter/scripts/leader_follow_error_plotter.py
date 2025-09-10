#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32
    
class ErrorRecorder:
    def __init__(self):
        # 获取话题名称参数
        self.e_d_topic = rospy.get_param('~e_d_topic', '/robot2/e_d')
        self.e_theta_topic = rospy.get_param('~e_theta_topic', '/robot2/e_theta')
    
        # 调试计数器
        self.e_d_count = 0
        self.e_theta_count = 0
    
        # 配置存储路径
        self.save_dir = os.path.join(os.path.expanduser("~"), "robot_error_plots")
        try:
            if not os.path.exists(self.save_dir):
                os.makedirs(self.save_dir)
                rospy.loginfo("Successfully created directory: %s", self.save_dir)
            else:
                rospy.loginfo("Using existing directory: %s", self.save_dir)
        except Exception as e:
            rospy.logerr("Directory creation failed: %s", str(e))
            sys.exit(1)
    
        # 数据容器
        self.e_d_data = []
        self.e_theta_data = []
    
        # 动态配置订阅者
        rospy.Subscriber(self.e_d_topic, Float32, self.e_d_cb, queue_size=10)
        rospy.Subscriber(self.e_theta_topic, Float32, self.e_theta_cb, queue_size=10)
    
        rospy.loginfo("Subscribers initialized:\n- %s\n- %s", 
                        self.e_d_topic, self.e_theta_topic)
    
    def e_d_cb(self, msg):
        """处理横向误差数据"""
        self.e_d_data.append(msg.data)
        self.e_d_count += 1
        if self.e_d_count % 50 == 0:  # 调整为每50条记录一次
            rospy.logdebug("Received e_d data count: %d", self.e_d_count)
    
    def e_theta_cb(self, msg):
        """处理角度误差数据"""
        self.e_theta_data.append(msg.data)
        self.e_theta_count += 1
        if self.e_theta_count % 50 == 0:
            rospy.logdebug("Received e_theta data count: %d", self.e_theta_count)
    
    def export_plot(self):  # 必须保留这个方法
        """安全保存方法"""
        try:
            rospy.loginfo("Shutdown triggered, starting export process...")
            
            # 数据存在性检查
            if not self.e_d_data and not self.e_theta_data:
                rospy.logwarn("No data collected, nothing to save")
                return

            # 生成合法文件名（新增处理函数）
            def sanitize_topic_name(topic_str):
                import re
                sanitized = re.sub(r'[^\w-]', '_', topic_str)
                sanitized = re.sub(r'_+', '_', sanitized)
                return sanitized.strip('_')

            timestamp = time.strftime("%Y%m%d_%H%M%S")
            base_name = "{}_and_{}_{}".format(
                sanitize_topic_name(self.e_d_topic),
                sanitize_topic_name(self.e_theta_topic),
                timestamp
            )
            filename = "error_plot_{}.png".format(base_name)
            save_path = os.path.join(self.save_dir, filename)
            
            rospy.loginfo("Attempting to save plot to: %s", save_path)
            
            # 创建图表
            fig, ax = plt.subplots(figsize=(12, 6))
            
            # 绘图逻辑
            if self.e_d_data:
                ax.plot(self.e_d_data, label='e_d', color='blue', linewidth=1.5)
            if self.e_theta_data:
                ax.plot(self.e_theta_data, label='e_theta', color='red', linewidth=1.5, linestyle='--')
            
            # 图表配置
            ax.set_title("Error Tracking\n({})".format(self.e_d_topic))
            ax.set_xlabel("Time Steps")
            ax.set_ylabel("Error Value")
            ax.legend()
            ax.grid(True, alpha=0.3)
            
            # 保存操作
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
            plt.close(fig)
            rospy.loginfo("Plot successfully saved to:\n%s", save_path)
            
        except Exception as e:
            rospy.logfatal("Save failed: %s", str(e))
            exc_type, exc_obj, exc_tb = sys.exc_info()
            rospy.logerr("Error at line %d", exc_tb.tb_lineno)
    
if __name__ == '__main__':
    try:
        rospy.init_node('error_recorder', log_level=rospy.INFO)
        recorder = ErrorRecorder()
        rospy.on_shutdown(recorder.export_plot)
        rospy.loginfo("Recorder initialized. Use Ctrl+C to save and exit.")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received")
    except Exception as e:
        rospy.logerr("Unhandled exception: %s", str(e))