# FoundationPose: Using with Runpod
## Pod Setup
Set up SSH to connect local machine to cloud terminal:
- https://docs.runpod.io/pods/configuration/use-ssh

Create Runpod template with the following settings:
- Container image (mention on original FoundationPose repo): 
```
shingarey/foundationpose_custom_cuda121:latest
```
- Container start command (from https://docs.runpod.io/pods/configuration/use-ssh):
```
bash -c 'apt update;DEBIAN_FRONTEND=noninteractive apt-get install openssh-server -y;mkdir -p ~/.ssh;cd $_;chmod 700 ~/.ssh;echo "$PUBLIC_KEY" >> authorized_keys;chmod 700 authorized_keys;service ssh start;sleep infinity'
```
Deploy and connect to terminal.
-	Deploy RTX 2000 Ada with foundation pose template.
-	Connect via SSH (in WSL terminal)
-	Enter the workspace: cd /workspace/

## FoundationPose Setup Inside Pod
-	Clone this repo: 
``` bash
git clone https://github.com/jacksonyoung4/Jackson-FoundationPose.git
```
-	Enter FoundationPose directory: 
``` bash
cd Jackson-FoundationPose
```
-	Download weights:
``` bash
pip install gdown
gdown --folder https://drive.google.com/drive/folders/1DFezOAD0oD1BblsXVxqDsl8fj0qzB82i
```
- Rename no_diffusion folder to weights:
``` bash
mv no_diffusion weights
```
-	Run for first time launching container: 
``` bash
bash build_all.sh
```
## Performing Pose Estimation
- Perform pose estimation:
``` bash
python run_demo.py --debug 2
```
- Send results to local machine:
``` bash
runpodctl send debug
```
