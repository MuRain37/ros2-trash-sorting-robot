import paho.mqtt.client as mqtt
import json
import threading
import time
from datetime import datetime
from config.config import MQTT_CONFIG, MQTT_TOPICS
from model.database import Database

class RobotMQTTClient:
    """机器人MQTT客户端"""
    
    def __init__(self):
        self.client = None
        self.connected = False
        self.message_queue = []
        self.callbacks = {}
        
        # 机器人状态数据
        self.robot_status = {
            'battery_level': 100.0,
            'temperature': 25.0,
            'humidity': 50.0,
            'position': {'x': 0, 'y': 0, 'theta': 0},
            'speed': 0.0,
            'status': 'idle',
            'error_code': 0,
            'error_message': ''
        }
        
        # 传感器数据
        self.sensor_data = {
            'ultrasonic': [],
            'infrared': [],
            'lidar': [],
            'temperature': 25.0,
            'humidity': 50.0
        }
        
        # 地图数据
        self.map_data = []
        
    def connect(self):
        """连接到MQTT代理"""
        try:
            self.client = mqtt.Client()
            self.client.username_pw_set(MQTT_CONFIG['username'], MQTT_CONFIG['password'])
            
            # 设置回调函数
            self.client.on_connect = self._on_connect
            self.client.on_message = self._on_message
            self.client.on_disconnect = self._on_disconnect
            
            # 连接代理
            self.client.connect(MQTT_CONFIG['broker'], MQTT_CONFIG['port'], MQTT_CONFIG['keepalive'])
            
            # 启动MQTT循环
            self.client.loop_start()
            print("MQTT客户端已启动")
            
            # 等待连接建立
            time.sleep(2)
            return self.connected
            
        except Exception as e:
            print(f"MQTT连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开MQTT连接"""
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
            self.connected = False
            print("MQTT客户端已断开")
    
    def _on_connect(self, client, userdata, flags, rc):
        """连接回调"""
        if rc == 0:
            self.connected = True
            print("MQTT连接成功")
            
            # 订阅所有主题
            for topic_name, topic in MQTT_TOPICS.items():
                client.subscribe(topic)
                print(f"已订阅主题: {topic}")
        else:
            print(f"MQTT连接失败，错误码: {rc}")
    
    def _on_message(self, client, userdata, msg):
        """消息接收回调"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            
            print(f"收到消息 - 主题: {topic}, 数据: {data}")
            
            # 根据主题处理数据
            if topic == MQTT_TOPICS['robot_status']:
                self._handle_robot_status(data)
            elif topic == MQTT_TOPICS['robot_sensors']:
                self._handle_sensor_data(data)
            elif topic == MQTT_TOPICS['robot_position']:
                self._handle_position_data(data)
            elif topic == MQTT_TOPICS['robot_lidar']:
                self._handle_lidar_data(data)
            elif topic == MQTT_TOPICS['robot_camera']:
                self._handle_camera_data(data)
            
            # 调用注册的回调函数
            if topic in self.callbacks:
                for callback in self.callbacks[topic]:
                    callback(data)
                    
        except Exception as e:
            print(f"处理MQTT消息时出错: {e}")
    
    def _on_disconnect(self, client, userdata, rc):
        """断开连接回调"""
        self.connected = False
        print(f"MQTT连接断开，错误码: {rc}")
    
    def _handle_robot_status(self, data):
        """处理机器人状态数据"""
        self.robot_status.update(data)
        
        # 保存到数据库
        query = """
            INSERT INTO robot_status 
            (battery_level, temperature, humidity, position_x, position_y, 
             orientation, speed, status, error_code, error_message)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
        """
        
        position = data.get('position', {})
        params = (
            data.get('battery_level', 0),
            data.get('temperature', 0),
            data.get('humidity', 0),
            position.get('x', 0),
            position.get('y', 0),
            position.get('theta', 0),
            data.get('speed', 0),
            data.get('status', 'unknown'),
            data.get('error_code', 0),
            data.get('error_message', '')
        )
        
        Database.execute_query(query, params)
    
    def _handle_sensor_data(self, data):
        """处理传感器数据"""
        sensor_type = data.get('sensor_type', 'unknown')
        sensor_id = data.get('sensor_id', 'unknown')
        value = data.get('value', 0)
        unit = data.get('unit', '')
        
        # 更新内存中的数据
        if sensor_type in self.sensor_data:
            if isinstance(self.sensor_data[sensor_type], list):
                self.sensor_data[sensor_type].append({
                    'value': value,
                    'timestamp': datetime.now().isoformat()
                })
                # 只保留最近100条数据
                if len(self.sensor_data[sensor_type]) > 100:
                    self.sensor_data[sensor_type] = self.sensor_data[sensor_type][-100:]
            else:
                self.sensor_data[sensor_type] = value
        
        # 保存到数据库
        query = """
            INSERT INTO sensor_data (sensor_type, sensor_id, value, unit)
            VALUES (%s, %s, %s, %s)
        """
        params = (sensor_type, sensor_id, value, unit)
        Database.execute_query(query, params)
    
    def _handle_position_data(self, data):
        """处理位置数据"""
        if 'position' in data:
            self.robot_status['position'] = data['position']
    
    def _handle_lidar_data(self, data):
        """处理激光雷达数据"""
        if 'scan_data' in data:
            self.map_data = data['scan_data']
            
            # 保存到数据库（简化版，实际可能需要批量插入）
            query = """
                INSERT INTO map_data (grid_x, grid_y, value, data_type)
                VALUES (%s, %s, %s, %s)
            """
            
            # 清空旧数据
            Database.execute_query("DELETE FROM map_data WHERE data_type = 'lidar'")
            
            # 插入新数据（这里简化处理，实际应该批量插入）
            for i, distance in enumerate(data['scan_data']):
                if i % 10 == 0:  # 每10个点保存一个，减少数据量
                    angle = i * (360 / len(data['scan_data']))
                    x = distance * math.cos(math.radians(angle))
                    y = distance * math.sin(math.radians(angle))
                    
                    grid_x = int(x / 0.1)  # 假设网格大小为0.1米
                    grid_y = int(y / 0.1)
                    
                    params = (grid_x, grid_y, distance, 'lidar')
                    Database.execute_query(query, params)
    
    def _handle_camera_data(self, data):
        """处理摄像头数据"""
        # 这里可以保存摄像头帧到数据库
        pass
    
    def publish_command(self, command_type, command_data):
        """发布控制命令"""
        if not self.connected:
            print("MQTT未连接，无法发布命令")
            return False
        
        try:
            topic = MQTT_TOPICS.get(command_type)
            if not topic:
                print(f"未知的命令类型: {command_type}")
                return False
            
            payload = json.dumps(command_data)
            result = self.client.publish(topic, payload)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f"命令已发布 - 主题: {topic}, 数据: {command_data}")
                
                # 记录操作日志
                query = """
                    INSERT INTO operation_logs (user_id, action_type, action_details)
                    VALUES (%s, %s, %s)
                """
                params = ('web_user', command_type, json.dumps(command_data))
                Database.execute_query(query, params)
                
                return True
            else:
                print(f"命令发布失败，错误码: {result.rc}")
                return False
                
        except Exception as e:
            print(f"发布命令时出错: {e}")
            return False
    
    def register_callback(self, topic, callback):
        """注册主题回调函数"""
        if topic not in self.callbacks:
            self.callbacks[topic] = []
        self.callbacks[topic].append(callback)
    
    def get_robot_status(self):
        """获取机器人状态"""
        return self.robot_status
    
    def get_sensor_data(self):
        """获取传感器数据"""
        return self.sensor_data
    
    def get_map_data(self):
        """获取地图数据"""
        return self.map_data

# 全局MQTT客户端实例
mqtt_client = RobotMQTTClient()

def init_mqtt_client():
    """初始化MQTT客户端"""
    return mqtt_client.connect()

if __name__ == "__main__":
    # 测试MQTT客户端
    import math
    
    client = RobotMQTTClient()
    if client.connect():
        print("MQTT客户端测试成功")
        
        # 测试发布命令
        test_command = {
            'command': 'move_to',
            'target': {'x': 5, 'y': 3},
            'speed': 0.5
        }
        client.publish_command('robot_control', test_command)
        
        time.sleep(5)
        client.disconnect()
    else:
        print("MQTT客户端测试失败")