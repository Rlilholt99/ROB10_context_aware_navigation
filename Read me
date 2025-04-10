To run Mask2Former you need to install some stuff like:

* transformers and torch

i will highly recommend using the GPU since this is mainly convolution in this case you would need to get torchcuda:
follow the guid and get CUAD TOOlkit installed:
https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_local


Once you have cuda up and runing next you need to do is to make sure you are runing your Gazebo sim in this case the tiago-base:

$ ros2 launch tiago_gazebo tiago_gazebo.launch.py navigation:=False is_public_sim:=True [arm_type:=no-arm]

Once it start runing and you wan make sure that everything is in orde look in:

$ ros2 topic list

in here you should be able to see 2 topics that we are intersed in camera RGB and camera depth.



After you made sure everything is in order run this code to do the Mask2Former panoptic segmentations:
ros2 run stereo_panoptic_segmentation SPS_node


Then to see what going on in the image and how it's been segmenated use this
ros2 run rqt_image_view rqt_image_view

And to see how fast we are segmentating we can use this:

ros2 topic hz /segmentation/annotated


it should be between 6.5-7.9 hz ish
