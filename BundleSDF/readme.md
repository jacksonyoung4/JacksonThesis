# BundleSDF: Using with Runpod
## Pod Setup
Set up SSH to connect local machine to cloud terminal (if not already set up):
- https://docs.runpod.io/pods/configuration/use-ssh

## Deploy Pod
### Using Existing Template
Deploy Pod with BundleSDF template used in this thesis:
- Click this link and deploy with RTX 2000 Ada or other suitable GPU: https://console.runpod.io/deploy?template=hldawr6u87&ref=4jgva10z

### Create Template from Scratch
Alternatively, create BundleSDF template from scratch with the following settings:
- Container Image: 
```
jacksonyoung4/bundlesdf:2025-09-21
```
- Container Start Command (from https://docs.runpod.io/pods/configuration/use-ssh):
```
bash -c 'apt update;DEBIAN_FRONTEND=noninteractive apt-get install openssh-server -y;mkdir -p ~/.ssh;cd $_;chmod 700 ~/.ssh;echo "$PUBLIC_KEY" >> authorized_keys;chmod 700 authorized_keys;service ssh start;sleep infinity'
```
- Add some persistent storage (Volume Disk > 5GB)
- All other settings can remain default.

### Connect to Pod
-	Connect via SSH (in WSL terminal)
-	Enter Pod workspace: 
```
cd /workspace/
```
## BundleSDF Setup Inside Pod
-	Clone this repo: 
``` bash
git clone https://github.com/jacksonyoung4/JacksonThesis.git
```
-	Enter BundleSDF directory: 
``` bash
cd JacksonThesis/BundleSDF
```
-	Run if first time launching container: 
``` bash
bash build.sh
```
-	Download pretrained weights:
``` bash
pip install gdown
cd BundleTrack/LoFTR
mkdir weights
cd weights
gdown https://drive.google.com/uc?id=1M-VD35-qdB5Iw-AtbDBCKC7hPolFW9UY
```
## Working with Custom Data
Record object using record_for_BundleSDF.py from RealSense directory.
### Obtain Masks with XMem
For BundleSDF, a mask of the object is required for each image frame.   
These can be generated with XMem: https://github.com/hkchengrex/XMem
-	In WSL: 
``` bash
git clone https://github.com/hkchengrex/XMem.git
```
-	Install requirements described in: getting_started.md
-	Download pretrained models described in: inference.md
-	Add recording in same folder as XMem repository.
-	Data must have the following folder structure (only first frame needed for annotations):
``` bash
├── custom_data_root
│   ├── JPEGImages
│   │   ├── video1
│   │   │   ├── 00001.jpg
│   │   │   ├── 00002.jpg
│   │   │   ├── ...
│   │   └── ...
│   ├── Annotations
│   │   ├── video1
│   │   │   ├── 00001.png
│   │   │   ├── ...
│   │   └── ...
```
- Data will be in this format by default if using record_for_BundleSDF.py.
- Run XMem:
``` bash
-	python3 eval.py --output ../output/dataName --generic_path ../dataName --dataset G
```
-	Result is located in output folder.

### Compile Files
- Taking the masks from the output of XMem, compile files in format:
```
object_name
  ├──rgb/    (PNG files)
  ├──depth/  (PNG files, stored in mm, uint16 format. Filename same as rgb)
  ├──masks/  (PNG files. Filename same as rgb. 0 is background. Else is foreground)
  └──cam_K.txt   (3x3 intrinsic matrix, use space and enter to delimit)
```

## Create Mesh
- Upload custom data to Google Drive and download into BundleSDF directory of Pod using gdown (and unzip).
-	Run joint tracking and reconstruction: 
``` bash
python run_custom.py --mode run_video --video_dir object_name --out_folder /workspace/object_name --use_segmenter 1 --use_gui 0 --debug_level 2
```
- Run global refinement postprocessing to refine the mesh: 
```bash
python run_custom.py --mode global_refine --video_dir object_name --out_folder /workspace/object_name
```
- Send results to local machine:
``` bash
runpodctl send object_name
```

## Troubleshooting

- If 
"ImportError: /usr/lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.29' not found (required by /opt/conda/envs/py38/lib/python3.8/site-packages/scipy/spatial/_ckdtree.cpython-38-x86_64-linux-gnu.so)"
``` bash
pip uninstall scipy
pip uninstall scipy
pip install scipy
```
