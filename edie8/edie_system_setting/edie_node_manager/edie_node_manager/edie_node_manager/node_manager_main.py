import rclpy
import yaml
import re
import os
import subprocess
import datetime
from queue import Queue
from rclpy.node import Node
from edie_node_manager.msg import RunningNodes
from edie_node_manager.msg import TerminatedNodes

class NodeManager(Node):
    def __init__(self):
        super().__init__('edie_node_manager')

        # Get Environment Variable
        self.pkg_name = "edie_node_manager"
        self.ROS_WS = os.getenv('ROS_WS', '~/ros2_ws')

        # ROS2 Publisher
        self.pub_running_nodes = self.create_publisher(RunningNodes, '/edie8/node_manager/running_nodes', 1)
        self.pub_terminated_nodes = self.create_publisher(TerminatedNodes, '/edie8/node_manager/terminated_nodes', 1)
        # self.sub_running_nodes_xav = self.create_subscription(RunningNodes, '/running_nodes_xav', self.sub_running_xav_callback, 10)
        # self.sub_terminated_nodes_xav = self.create_subscription(TerminatedNodes, '/terminated_nodes_xav', self.sub_terminated_xav_callback, 10)
        
        self.timer_period = 3
        self.timer = self.create_timer(self.timer_period, self.compare_node_list)
        self.config_file_full_path = f"{self.ROS_WS}/src/edie8/edie_system_setting/edie_node_manager/{self.pkg_name}/config/node_configure.yaml"
        self.yaml_data = self.load_yaml_data()
        self.registered_nodes = self.get_registered_nodes_list()
        
        # 실행중인 노드
        self.running_nodes = []
        # 종료된 노드 (약속된 내용과 비교한 결과)
        self.missing_nodes = {}

        self.restart_waiting_queue = Queue()
        self.restart_waiting_set = set()
        self.initialize_log_file()
        self.print_registered_node_names()

    def load_yaml_data(self):
        with open(self.config_file_full_path, 'r') as file:
            yaml_data = yaml.safe_load(file)
        return yaml_data

    def get_registered_nodes_list(self):
        '''
        yaml로 부터 등록된 노드들 가져옴
        '''
        registered_nodes = []
        for package in self.yaml_data.values():
            for node in package[1:]:
                registered_nodes.append(node)
        return registered_nodes
    
    def get_running_nodes(self):
        '''
        실행중인 모든 노드들을 리스트화
        '''
        cmd_str = 'ps aux | grep install/ | grep lib/ | grep -v node_manager | grep -v edie8_gui'
        running_ps_names_str = subprocess.run(cmd_str, shell=True, executable='/bin/bash', text=True, capture_output=True)
        running_ps_names = running_ps_names_str.stdout.strip().split("\n")

        pattern = re.compile(r'/lib/([^/]+)/([^/\s]+)')

        self.running_nodes.clear()    
    
        if running_ps_names == ['']:
            running_ps_names = []

        if running_ps_names:
            for process in running_ps_names:
                match = pattern.search(process)
                if match:
                    self.running_nodes.append(match.group(2))
            return 0
        else:
            self.running_nodes = []
            return -1

    def print_registered_node_names(self):
        '''
        디버깅용
        등록된 노드들 출력
        '''
        self.get_logger().info("=====REGISTERED NODES=====")
        if self.registered_nodes is not None:
            for ele in self.registered_nodes:
                self.get_logger().info(ele)

    def print_running_node_names(self):
        '''
        디버깅용
        실행중인 노드들 출력
        '''
        self.get_logger().info("=====RUNNING NODES=====")
        if self.running_nodes is not None:
            for ele in self.running_nodes:
                self.get_logger().info(ele)

    def print_missing_node_names(self):
        '''
        디버깅용
        실행중인 노드들 출력
        '''
        self.get_logger().info("=====MISSING NODES=====")
        if self.missing_nodes is not None:
            for ele in self.missing_nodes:
                self.get_logger().info(ele)

    def publish_running_nodes(self):
        msg = RunningNodes()
        msg.node_name = self.running_nodes
        self.pub_running_nodes.publish(msg)

    def publish_terminated_nodes(self):
        msg = TerminatedNodes()
        msg.node_name = list(self.missing_nodes)
        self.pub_terminated_nodes.publish(msg)

    def compare_node_list(self):
        '''
        3초 마다 실행중인 노드와 등록된 노드 비교 후 이상시 재실행
        '''
        # if not self.get_running_nodes(): # 실행중인 노드가 있으면
        self.get_running_nodes()
        self.missing_nodes = set(self.registered_nodes) - set(self.running_nodes)

        if self.missing_nodes:
            self.log_missing_nodes(self.missing_nodes)
            self.get_logger().warn("The following nodes have not been executed. They will be executed within a few seconds.")
            for full_node_name in self.missing_nodes:
                self.get_logger().warn(f"   - Queueing {full_node_name} for restart")
                self.restart_waiting_queue.put(full_node_name)
                self.restart_waiting_set.add(full_node_name)
        else:
            self.print_greeny("All registered nodes are currently running.")
        # else: # 실행중인 노드가 없으면
        #     self.get_logger().error("Please execute the nodes before the node manager.")

        self.publish_running_nodes()
        self.publish_terminated_nodes()
        self.restart_nodes_from_queue()
        # self.print_running_node_names()

    def get_package_from_node(self, node_name):
        for package_name, nodes in self.yaml_data.items():
            if node_name in nodes:
                return package_name
        return ""

    def get_param_path_from_package(self, package_name):
        '''
        패키지의 첫 번째 항목이 PARAM_PATH임을 가정
        '''
        if package_name in self.yaml_data:
            param_path = self.yaml_data[package_name][0].get("PARAM_PATH", "No PARAM_PATH specified")
            param_path = param_path.replace("${ROS_WS}", self.ROS_WS)
            # self.get_logger().info(param_path)
            return param_path
        else:
            return ""

    def execute_command(self, pkg_name, node_name, param_path):

        if not param_path:
            cmd_str = f"""source /opt/ros/humble/setup.bash;
            source ~/ros2_ws/install/local_setup.bash;
            ros2 run {pkg_name} {node_name}"""
        else:
            cmd_str = f"""source /opt/ros/humble/setup.bash;
            source ~/ros2_ws/install/local_setup.bash;
            ros2 run {pkg_name} {node_name} --ros-args --params-file {param_path}"""

        # self.get_logger().info(cmd_str)
        subprocess.Popen(cmd_str, shell=True, executable='/bin/bash', stderr=subprocess.DEVNULL)

    def restart_nodes_from_queue(self):
        if not self.restart_waiting_queue.empty():
            full_node_name = self.restart_waiting_queue.get()
            self.restart_waiting_set.discard(full_node_name)
            node_name = full_node_name.split('/')[-1]

            if full_node_name not in self.running_nodes:
                pkg_name = self.get_package_from_node(node_name)
                param_path = self.get_param_path_from_package(pkg_name)

                self.execute_command(pkg_name, node_name, param_path)

    def initialize_log_file(self):
        '''
        파일 내용 초기화
        '''
        log_file_path = f"{self.ROS_WS}/src/edie8/edie_system_setting/edie_node_manager/{self.pkg_name}/log/terminated_nodes.txt"
        with open(log_file_path, "w") as log_file:
            log_file.write("")  

    def log_missing_nodes(self, missing_nodes):
        log_file_path = f"{self.ROS_WS}/src/edie8/edie_system_setting/edie_node_manager/{self.pkg_name}/log/terminated_nodes.txt"

        with open(log_file_path, "a") as log_file:
            current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            if missing_nodes:
                log_file.write(f"\n[{current_time}] 다음 노드들이 실행되지 않았습니다:\n")
                for full_node_name in missing_nodes:
                    log_message = f"   - {full_node_name}\n"
                    log_file.write(log_message)

    def print_greeny(self, text):
        self.get_logger().info("\033[1;92m" + text + "\033[0m")

def main(args=None):
    rclpy.init(args=args)
    node = NodeManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()