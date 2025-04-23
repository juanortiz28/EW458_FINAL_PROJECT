import roslibpy
import time
from pynput import keyboard
import math
import numpy as np

class KeyboardController:
    def __init__(self, robot_name, ros_ip, ros_port):
        self.robot_name = robot_name
        self.ros = roslibpy.Ros(host=ros_ip, port=ros_port)
        self.ros.run()
        # ROS Topics
        self.drive_topic = roslibpy.Topic(self.ros, f'/{robot_name}/cmd_vel', 'geometry_msgs/msg/Twist')
        self.odom_topic = roslibpy.Topic(self.ros, f'/{robot_name}/odom', 'nav_msgs/msg/Odometry')
        self.scan_topic = roslibpy.Topic(self.ros, f'/{robot_name}/scan', 'sensor_msgs/msg/LaserScan')
        self.map_topic = roslibpy.Topic(self.ros, f'/{robot_name}/saltymap', 'nav_msgs/msg/OccupancyGrid')
        self.reset_pose = roslibpy.Service(self.ros, f'/{robot_name}/reset_pose', 'irobot_create_msgs/ResetPose')
        # Subscribe to topics
        # self.reset_pose.call(roslibpy.ServiceRequest())
        self.odom_topic.subscribe(self.odom_callback)
        self.scan_topic.subscribe(self.scan_callback)
        # States
        self.armed = False
        self.movement = {"forward": False, "left": False, "right": False}
        self.pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.scan_info = {}
        # Keyboard
        self.keyboard = keyboard.Listener(on_press=self.key_down, on_release=self.key_up)
        self.keyboard.start()
        # MAP 50row x 100col each box is 0.1m x 0.1m so it is a 10m x 5m map in robot coordinates
        self.width = 500
        self.height = 500
        self.resolution = 0.1
        self.map = np.full((self.height, self.width), -1)
        self.origin = (-self.width//2 * self.resolution, -self.height//2 * self.resolution - 2)
        self.binMap = np.zeros((self.height, self.width), dtype=int)
        self.updating = False
        self.last_map_update = time.time()
        self.map_update_interval = 0.2  # seconds

    def quaternion_to_yaw(self, q):
        x, y, z, w = q['x'], q['y'], q['z'], q['w']
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def odom_callback(self, msg):
        pos = msg['pose']['pose']['position']
        ori = msg['pose']['pose']['orientation']
        self.pose['x'] = pos['x']
        self.pose['y'] = pos['y']
        self.pose['theta'] = self.quaternion_to_yaw(ori)

    def scan_callback(self, msg): 
        # Update scan info
        # meters
        self.scan_info = {
            "angle_increment": msg['angle_increment'],
            "angle_min": msg['angle_min'],
            "angle_max": msg['angle_max'],
            "range_min": msg['range_min'],
            "range_max": msg['range_max'],
            "ranges": msg['ranges']
        }

    def drive(self, v, w):
        self.drive_topic.publish(roslibpy.Message({
            'linear': {'x': v, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': w}
        }))


    def key_down(self, key):
        if key == keyboard.Key.space:
            self.armed = not self.armed
            print("ARMED" if self.armed else "DISARMED")
        elif hasattr(key, 'char') and key.char:
            if key.char == 'q':
                self.ros.terminate()
                return False
            elif key.char == 'r':
                self.map = np.full((self.height, self.width), -1)
                self.origin = (-self.width//2 * self.resolution, -self.height//2 * self.resolution)
                self.reset_pose.call(roslibpy.ServiceRequest())
                print("Pose reset")
            elif key.char == 'g':
                self.updating = not self.updating
                print("Auto-map updating ON" if self.updating else "Auto-map updating OFF")
            elif key.char == 't':
                    self.update_pose = self.pose
                    self.update_scan_info = self.scan_info
                    self.update_map()
                    print("Map updated")    
            elif key.char == 'w':
                self.movement["forward"] = True
            elif key.char == 'a':
                self.movement["left"] = True
            elif key.char == 'd':
                self.movement["right"] = True

    def key_up(self, key):
        if hasattr(key, 'char') and key.char:
            if key.char == 'w':
                self.movement["forward"] = False
            elif key.char == 'a':
                self.movement["left"] = False
            elif key.char == 'd':
                self.movement["right"] = False

    def drive_loop(self):
        if self.armed:
            v = 0.3 if self.movement["forward"] else 0
            w = .5 if self.movement["left"] else -0.5 if self.movement["right"] else 0
            self.drive(v, w)

            if self.updating and (time.time() - self.last_map_update) >= self.map_update_interval:
                if not self.movement["left"] and not self.movement["right"]:
                    # Only update the map if the robot is not moving
                    self.update_pose = self.pose
                    self.update_scan_info = self.scan_info
                    self.update_map()
                    self.last_map_update = time.time()
                    print("Map updated continuously")


    def m2grid(self, x, y):
        # Convert METERS from robot coordinates to grid coordinates in matrix
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)

        return gx, gy
    
    def binaryMap(self):
        updateB = np.zeros((self.height, self.width), dtype=int)

        if not self.update_scan_info:
            print("No scan info available.")
            return updateB

        for i, r in enumerate(self.update_scan_info['ranges']):
            if r < self.update_scan_info['range_min'] or r > self.update_scan_info['range_max']:
                continue

            map_angle = self.update_scan_info['angle_min'] + i * self.update_scan_info['angle_increment']
            world_angle = self.update_pose['theta'] + map_angle 

            hit_x = self.update_pose['x'] + r * math.cos(world_angle)
            hit_y = self.update_pose['y'] + r * math.sin(world_angle)

            rx, ry = self.m2grid(self.update_pose['x'], self.update_pose['y'])
            # # Mark robot position in map (for debug)
            # rx, ry = self.m2grid(self.pose['x'], self.pose['y'])
            # if 0 <= rx < self.width and 0 <= ry < self.height:
            #     updateB[ry][rx] =   # Arbitrary marker for robot position

            hx, hy = self.m2grid(hit_x, hit_y)

            # print(f"Laser angle: {map_angle:.2f} | World angle: {world_angle:.2f} | Robot theta: {self.update_pose['theta']:.2f}")
            # print(f"Robot (m): ({self.update_pose['x']:.2f}, {self.update_pose['y']:.2f}) -> Grid: ({rx}, {ry})")
            # print(f"Hit (m):   ({hit_x:.2f}, {hit_y:.2f}) -> Grid: ({hx}, {hy})")

            # Step through the line from robot to hit point
            steps = max(abs(hx - rx), abs(hy - ry))
            if steps == 0:
                continue  # Avoid division by zero

            for step in range(steps):
                t = step / steps
                px = round(rx + t * (hx - rx))
                py = round(ry + t * (hy - ry))
                if 0 <= px < self.width and 0 <= py < self.height:
                    updateB[py][px] = -1  # Free space between robot and hit

            # Obstacle cell
            if 0 <= hx < self.width and 0 <= hy < self.height:
                updateB[hy][hx] = 20  # Obstacle

        return updateB

    def update_map(self):
        # Get the binary map update
        updateB = self.binaryMap()
        special_cases = (self.map == 0) & (updateB == -1)
        self.map = np.where(special_cases, self.map, self.map + updateB)
        self.map = np.where(self.map < -1, 0, np.clip(self.map, -1, 100))

        grid_msg = {
            'header': {
                'stamp': {'sec': int(time.time()), 'nanosec': 0},
                'frame_id': 'map'
            },
            'info': {
                'map_load_time': {'sec': int(time.time()), 'nanosec': 0},
                'resolution': self.resolution,
                'width': self.width,
                'height': self.height,
                'origin': {
                    'position': {'x': self.origin[0], 'y': self.origin[1], 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            },
            'data': self.map.flatten().tolist()
        }
        self.map_topic.publish(roslibpy.Message(grid_msg))

    def run(self):
        try:
            while True:
                self.drive_loop()
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Exiting...")
        finally: 
            self.drive(0, 0)
            self.ros.terminate()

if __name__ == "__main__":
    robot_name = "echo"
    ros_ip = "192.168.8.104"
    ros_port = 9012
    controller = KeyboardController(robot_name, ros_ip, ros_port)
    controller.run()