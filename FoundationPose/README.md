# FoundationPose: Using with Runpod
## Pod Setup
Set up SSH to connect local machine to cloud terminal (if not already set up):
- https://docs.runpod.io/pods/configuration/use-ssh

## Deploy Pod
### Using Existing Template
Deploy Pod with FoundationPose template used in this thesis:
- Click this link and deploy with RTX 2000 Ada or other suitable GPU: https://console.runpod.io/deploy?template=8fqy086ds2&ref=4jgva10z

### Create Template from Scratch
Alternatively, create FoundationPose template from scratch with the following settings:
- Container image: 
```
shingarey/foundationpose_custom_cuda121:latest
```
- Container start command (from https://docs.runpod.io/pods/configuration/use-ssh):
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

## FoundationPose Setup Inside Pod
-	Clone this repo: 
``` bash
git clone https://github.com/jacksonyoung4/JacksonThesis.git
```
-	Enter FoundationPose directory: 
``` bash
cd JacksonThesis/FoundationPose
```
-	Download pretrained weights:
``` bash
pip install gdown
gdown --folder https://drive.google.com/drive/folders/1DFezOAD0oD1BblsXVxqDsl8fj0qzB82i
```
- Rename no_diffusion folder to weights:
``` bash
mv no_diffusion weights
```
-	Run if first time launching container: 
``` bash
bash build_all.sh
```
## Working with Custom Data
- Generate object mesh by following README in BundleSDF directory.
- Record object using record_for_FoundationPose.py from RealSense directory for pose estimation and tracking.
- Compile files. Must include RGB, depth, mask (first frame only), mesh, and intrinsic matrix of camera in format:
```
test_data
  ├──rgb/    (PNG files)
  ├──depth/  (PNG files, stored in mm, uint16 format. Filename same as rgb)
  ├──masks/  (PNG files. Filename same as rgb. 0 is background. Else is foreground)
  ├──mesh/   (Mesh files: .obj, .mtl and .png. .obj file must be named "textured_mesh")
  └──cam_K.txt   (3x3 intrinsic matrix, use space and enter to delimit)
```
- Upload custom data to Google Drive and download into FoundationPose directory of Pod using gdown (and unzip).

## Performing Pose Estimation
- Perform pose estimation:
``` bash
python run_demo.py --debug 2
```
- Send results to local machine:
``` bash
runpodctl send debug
```
