# GNetDetection

GNetDetection application using GTI 2803 Dongle.

To run the applications, setting up library path is required:

```
export LD_LIBRARY_PATH=${GTISDKPATH}/Lib/${OS_TYPE}/${CPU_ARCH}/
```

## Getting Started

1. Make sure you have GTI 2803 AI Accelerator and installed GTI SDK.
2. Prepare Build Tools and Python Default Packages
   ```shell
   sudo apt update && sudo apt install -y build-essential cmake python3-dev python3-venv python3-pip
   ```
3. Prepare Virtual Env
   > Pathnames must not contain multibyte characters
   ```shell 
   python3 -mvenv venv
   . venv/bin/activate
   pip install --upgrade wheel setuptools pip
   ```
4. Install dependencies
   ```shell 
   . venv/bin/activate   
   pip install -r requirements.txt
   ```
5. Run the demo (Sample commands)
   ```shell
   python3 run_gti_demo.py  image  ../../assets/models/2803/2803_gnetdet.model ../../assets/data/detection_data/test.jpg
   python3 run_gti_demo.py  image  ../../assets/models/2803/2803_gnetdet.model ../../assets/data/detection_data/street.jpg
   python3 run_gti_demo.py  video  ../../assets/models/2803/2803_gnetdet.model ../../assets/data/detection_data/test.mp4
   python3 run_gti_demo.py  camera ../../assets/models/2803/2803_gnetdet.model 0
   ```
6. Deactivate venv
   > After using the example, venv must be deactivated.
   ```shell
   deactivate
   ```

## GNetDetection descriptions and usage

Control key guidance
q: quit
Space: Pause

target options: image/video/camera

## For OpenCV autocompletion in PyCharm

```shell
. venv/bin/activate
export site_package=$(python -c 'import site; print(site.getsitepackages()[0])')
ln -s $site_package/cv2/cv2.abi3.so $site_package/cv2.abi3.so
```
