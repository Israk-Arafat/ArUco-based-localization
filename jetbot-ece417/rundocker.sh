docker run -it \
	--runtime nvidia \
	--network host \
	--privileged \
	--device /dev/video* \
	-v /dev/bus/usb:/dev/bus/usb \
	-v /tmp/argus_socket:/tmp/argus_socket \
	-v /home/jetbot:/home/jetbot \
	--workdir /home/jetbot/ece417 \
	--name=ros-humble \
	vdhiman86/ros:humble-pytorch-l4t-r32.7.1
