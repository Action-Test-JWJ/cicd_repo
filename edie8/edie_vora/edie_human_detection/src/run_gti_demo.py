#!venv/bin/python3

import sys
from typing import List

from demo import Demo
from gyurinet.gtilib import VisionTypeEnum


def print_usage(argv: List[str]):
    model_path = '../../assets/models/2803/gti_gnetdet_2803.model'

    s = ""
    s += f"Usage: python3 {argv[0]}  command   model_file {' ' * 40}[image/video/dir/...]\n"
    s += f"       python3 {argv[0]}  image     {model_path} ../../assets/data/detection_data/test.jpg\n"
    s += f"       python3 {argv[0]}  image     {model_path} ../../assets/data/detection_data/street.jpg\n"
    s += f"       python3 {argv[0]}  video     {model_path} ../../assets/data/detection_data/test.mp4\n"
    s += f"       python3 {argv[0]}  camera    {model_path} 0\n"
    print(s)
    sys.exit(-1)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_usage(sys.argv)

    demo = Demo(VisionTypeEnum(sys.argv[1]), sys.argv[2], sys.argv[3])
    demo.run()
