#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sqlite3
import os
import rospy

class WalleDatabase:
    def __init__(self):
        # 在当前代码目录下自动生成数据库文件
        self.db_path = os.path.join(os.path.dirname(__file__), "robot_system.db")
        self.init_db()

    def init_db(self):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            # 创建日志表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS Task_Log (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    target_type TEXT,
                    action TEXT,
                    status TEXT
                )
            ''')
            conn.commit()
            conn.close()
            rospy.loginfo("[数据库] SQLite3 初始化成功！")
        except Exception as e:
            rospy.logerr(f"[数据库] 初始化失败: {e}")

    def log_task(self, target_type, action, status):
        """记录任务日志"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute(
                "INSERT INTO Task_Log (target_type, action, status) VALUES (?, ?, ?)", 
                (target_type, action, status)
            )
            conn.commit()
            conn.close()
            rospy.loginfo(f"[数据库] 写入记录 -> 目标:{target_type}, 动作:{action}, 状态:{status}")
        except Exception as e:
            rospy.logerr(f"[数据库] 写入失败: {e}")
