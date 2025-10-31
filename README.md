# JacksonThesis
This repository contains all the code used in this thesis. See README within each subdirectory for specific implementation instructions.
## Directory Structure
### RealSense
- Contains code for recording with RealSense depth camera.
- Adapted from: https://github.com/Kaivalya192/Object_Reconstruction/blob/main/rec_con_mask.py

### BundleSDF
- Contains code for generating object meshes required for FoundationPose.  
- Code sourced from an earlier commit of the NVIDIA BundleSDF repository.  
- Commit link: https://github.com/NVlabs/BundleSDF/tree/cf0610e3636bc42535c996f129161f44beeadc56
- README altered to include detailed instructions for setup with Runpod.

### FoundationPose
- Code sourced from the NVIDIA FoundationPose repository.  
- Commit link: https://github.com/NVlabs/FoundationPose/tree/e3d597b8c6b851d053094ebd6fa240191c5238f8  
- README altered to include detailed instructions for setup with Runpod.

### BaxterCurtin
- Code for working with Baxter robot at Curtin.
- Created by Nathan Broadbent with code from previous thesis students.
- Repository link: https://github.com/nathan-broadbent/BaxterCurtin
- Code added for hand-eye calibration and pick-and-place tasks of this thesis.