import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import Qt
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory
import yaml
import subprocess

ui_name = "/mainwindow.ui"
logo_name = '/reg.png'
pkg_path = get_package_share_directory("edie_node_manager")
ui_dir_path = pkg_path + '/ui'
ui_path_file_path = ui_dir_path + ui_name
logo_img_path = ui_dir_path + logo_name
from_class = uic.loadUiType(ui_path_file_path)[0]

class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("edie_node_manager")
        self.setWindowIcon(QIcon(logo_img_path))

        # Set Properties
        self.setFixedSize(self.size())
        self.move(650, 300)
        self.ws_path = self.get_ws_path()
        self.ws_path_line_edit.setText(self.ws_path)
        self.executable_nodes_count = 0
        self.registered_nodes_count = 0

        self.node_list_tree.setColumnWidth(0, 250)
        self.node_list_tree.setColumnWidth(1, 250)
        self.registered_list_tree.setColumnWidth(0, 220)
        self.registered_list_tree.setColumnWidth(1, 250)

        self.name_space_line_edit.textChanged.connect(self.name_space_line_edit_changed)

        self.node_list_refresh_button.clicked.connect(self.node_list_refresh_button_clicked)
        self.node_list_expand_all_button.clicked.connect(self.node_list_expand_all_button_clicked)
        self.node_list_add_all_button.clicked.connect(self.node_list_add_all_button_clicked)
        self.node_list_add_button.clicked.connect(self.node_list_add_button_clicked)

        self.registered_list_expand_all_button.clicked.connect(self.registered_list_expand_all_button_clicked)
        self.registered_list_delete_all_button.clicked.connect(self.registered_list_delete_all_button_clicked)
        self.registered_list_delete_button.clicked.connect(self.registered_list_delete_button_clicked)

        self.import_button.clicked.connect(self.import_file)
        self.export_button.clicked.connect(self.export_file)
        self.exit_button.clicked.connect(self.close)

        self.registered_list_tree.itemChanged.connect(self.on_item_changed)

        self.is_expanded_executable_node_list = False
        self.is_expanded_registered_list = False

        self.node_param_path_dict = {}
        # [TODO]name_space 추후에 dict로 바꾸기
        self.name_space = ""

    def name_space_line_edit_changed(self):
        self.name_space = self.name_space_line_edit.text()

    def node_list_refresh_button_clicked(self):
        self.node_list_tree.clear()
        self.get_executables_list()

    def node_list_expand_all_button_clicked(self):
        if self.is_expanded_executable_node_list:
            self.node_list_tree.collapseAll()
            self.node_list_expand_all_button.setText("+")
        else:
            self.node_list_tree.expandAll()
            self.node_list_expand_all_button.setText("-")
        self.is_expanded_executable_node_list = not self.is_expanded_executable_node_list

    def node_list_add_all_button_clicked(self):
        self.registered_list_tree.clear()

        root = self.node_list_tree.invisibleRootItem()
        node_count = root.childCount()

        for i in range(node_count):
            item = root.child(i)
            pkg_item = QTreeWidgetItem(self.registered_list_tree)
            pkg_item.setText(0, item.text(0))

            # Add checkbox to package item
            pkg_item.setFlags(pkg_item.flags() | Qt.ItemIsUserCheckable)
            pkg_item.setCheckState(2, Qt.Unchecked)  # 체크박스는 기본적으로 체크되지 않은 상태

            child_count = item.childCount()
            for j in range(child_count):
                child = item.child(j)
                exec_item = QTreeWidgetItem(pkg_item)
                exec_item.setText(1, child.text(1))

        self.registered_nodes_count = self.executable_nodes_count
        self.registered_list_count_label.setText(f'Registered Nodes Count : {self.registered_nodes_count}')

    def node_list_add_button_clicked(self):
        selected_items = self.node_list_tree.selectedItems()
        if not selected_items:
            return  # No package selected

        selected_pkg = selected_items[0]
        if selected_pkg.parent():
            return  # This is not a package item, but a child node

        # Check for duplicates
        root = self.registered_list_tree.invisibleRootItem()
        for i in range(root.childCount()):
            if root.child(i).text(0) == selected_pkg.text(0):
                return  # Package already exists in registered_list_tree

        # Add the selected package to registered_list_tree
        pkg_item = QTreeWidgetItem(self.registered_list_tree)
        pkg_item.setText(0, selected_pkg.text(0))
        pkg_item.setFlags(pkg_item.flags() | Qt.ItemIsUserCheckable)
        pkg_item.setCheckState(2, Qt.Unchecked)  # 체크박스는 기본적으로 체크되지 않은 상태

        child_count = selected_pkg.childCount()
        for j in range(child_count):
            child = selected_pkg.child(j)
            exec_item = QTreeWidgetItem(pkg_item)
            exec_item.setText(1, child.text(1))

    def registered_list_expand_all_button_clicked(self):
        if self.is_expanded_registered_list:
            self.registered_list_tree.collapseAll()
            self.registered_list_expand_all_button.setText("+")
        else:
            self.registered_list_tree.expandAll()
            self.registered_list_expand_all_button.setText("-")
        self.is_expanded_registered_list = not self.is_expanded_registered_list

    def registered_list_delete_all_button_clicked(self):
        self.registered_list_tree.clear()
        self.node_param_path_dict.clear()

    def registered_list_delete_button_clicked(self):
        selected_items = self.registered_list_tree.selectedItems()

        if not selected_items:
            return  # 선택된 항목이 없으면 아무것도 하지 않음

        for item in selected_items:
            if item.parent():  # 선택된 항목이 부모(패키지)를 가지면, 노드 항목임
                # 노드만 삭제
                parent_item = item.parent()
                index = parent_item.indexOfChild(item)
                parent_item.takeChild(index)

                # 부모 항목(패키지)에 자식 노드가 더 이상 없으면 부모 항목도 삭제
                if parent_item.childCount() == 0:
                    index = self.registered_list_tree.indexOfTopLevelItem(parent_item)
                    self.registered_list_tree.takeTopLevelItem(index)
            else:
                # 패키지 항목이므로 패키지와 그 아래의 모든 노드 삭제
                index = self.registered_list_tree.indexOfTopLevelItem(item)
                self.registered_list_tree.takeTopLevelItem(index)

                # 딕셔너리에서 삭제
                for item in selected_items:
                    package_name = item.text(0)

                    if package_name in self.node_param_path_dict:
                        del self.node_param_path_dict[package_name]  # 딕셔너리에서 해당 패키지 경로 삭제

    def import_file(self):
        name = QFileDialog.getOpenFileName(self, 'Import Configure File', './')
        if name[0]:
            with open(name[0], "r") as file:
                self.registered_list_tree.clear()
                yaml_data = file.read()
                self.load_yaml_and_update_tree(yaml_data)

    def load_yaml_and_update_tree(self, yaml_data):
        # 관리하는 Dictionary 클리어
        self.node_param_path_dict.clear()

        # YAML 데이터 파싱
        parsed_data = yaml.safe_load(yaml_data)

        # registered_list_tree 및 node_param_path_dict 초기화
        self.registered_list_tree.blockSignals(True)  # 신호 블록 시작
        self.registered_list_tree.clear()
        self.node_param_path_dict.clear()

        # NAME_SPACE 초기화
        name_space_value = ""

        # 파싱된 데이터를 트리 및 node_param_path_dict에 추가
        for package_name, nodes in parsed_data.items():
            package_item = QTreeWidgetItem(self.registered_list_tree)
            package_item.setText(0, package_name)

            param_path = ""
            for node in nodes:
                if isinstance(node, dict):
                    # PARAM_PATH와 NAME_SPACE 처리
                    for key, value in node.items():
                        if key == "PARAM_PATH":
                            param_path = value
                        elif key == "NAME_SPACE":
                            name_space_value = value  # NAME_SPACE 값 저장
                else:
                    # 노드 항목 추가
                    node_item = QTreeWidgetItem(package_item)
                    node_item.setText(1, node)

            # 패키지 아이템에 체크박스 추가
            package_item.setFlags(package_item.flags() | Qt.ItemIsUserCheckable)
            # PARAM_PATH가 비어 있지 않으면 체크박스를 체크된 상태로 설정
            package_item.setCheckState(2, Qt.Checked if param_path else Qt.Unchecked)

            # PARAM_PATH가 있으면 node_param_path_dict에 추가
            if param_path:
                self.node_param_path_dict[package_name] = param_path

        # NAME_SPACE 설정
        if name_space_value:
            self.name_space_line_edit.setText(name_space_value)
            self.name_space = name_space_value  # 클래스 변수 업데이트

        self.registered_list_tree.blockSignals(False)  # 신호 블록 해제

    def export_file(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getSaveFileName(self, "Export Configure File", "node_configure.yaml",
                                                "YAML Files (*.yaml);;All Files (*)", options=options)
        if fileName:
            with open(fileName, 'w') as file:
                root = self.registered_list_tree.invisibleRootItem()
                for i in range(root.childCount()):
                    package_item = root.child(i)
                    package_name = package_item.text(0)
                    param_path = self.node_param_path_dict.get(package_name, "")
                    ns = self.name_space

                    file.write(f"{package_name}:\n")
                    file.write(f"  - PARAM_PATH: \"{param_path}\"\n")
                    file.write(f"  - NAME_SPACE: \"{ns}\"\n")

                    for j in range(package_item.childCount()):
                        node_item = package_item.child(j)
                        node_name = node_item.text(1)
                        file.write(f"  - {node_name}\n")

    ######################################################

    def get_executables_list(self):
        cmd_str = f'source /opt/ros/humble/setup.bash;ros2 pkg executables | grep edie | grep -v node_manager'

        node_list = subprocess.run(cmd_str, shell=True, executable='/bin/bash', text=True, capture_output=True)

        if node_list.stdout.strip():
            nodes_list = node_list.stdout.strip().split("\n")

            packages = {}
            for node in nodes_list:
                package_name, executable_name = node.split()
                if package_name not in packages:
                    packages[package_name] = []
                packages[package_name].append(executable_name)

            for package, executables in packages.items():
                pkg_item = QTreeWidgetItem(self.node_list_tree)
                pkg_item.setText(0, package)
                for exec_name in executables:
                    exec_item = QTreeWidgetItem(pkg_item)
                    exec_item.setText(1, exec_name)
        else:
            nodes_list = []
            print("Check node list again. There is not any node.")

        self.executable_nodes_count = len(nodes_list)
        self.executables_count_label.setText(f'Executable Nodes Count : {self.executable_nodes_count}')

    def get_ws_path(self):
        ws_path = os.environ.get('COLCON_PREFIX_PATH')
        modified_path = ws_path.rsplit('/install', 1)[0]
        return modified_path

    def on_item_changed(self, item, column):
        if column == 2:  # 체크박스가 있는 컬럼
            package_name = item.text(0)
            if item.checkState(column) == Qt.Checked:
                # print(f"{item.text(1)} is checked")
                file_name, _ = QFileDialog.getOpenFileName(self, 'Open File', self.ws_path + '/src')
                self.node_param_path_dict[package_name] = file_name
                if not file_name:  # 사용자가 'Cancel'을 눌렀을 경우
                    item.setCheckState(column, Qt.Unchecked)  # 체크박스 해제
            else:
                if package_name in self.node_param_path_dict:  # 체크 해제 시 딕셔너리에서 해당 항목 제거
                    del self.node_param_path_dict[package_name]

        # print(self.node_param_path_dict)

def main():
    app = QApplication(sys.argv)
    mywindows = WindowClass()
    mywindows.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()