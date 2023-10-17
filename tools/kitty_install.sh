

#Update
sudo apt update;sudo apt-get upgrade -y

#Realsense Camera
sudo apt install -y ros-humble-realsense2-camera ros-humble-compressed-image-transport

#Conetracking
sudo apt install -y python3-pip 
pip install ultrylytics
pip install numpy==1.24.1

#pip install sklearn
#pip install opencv-contrib-python


#Chrony Time Sync
sudo apt-get install chrony
echo "server 192.168.3.2 minpoll 0 maxpoll 5 maxdelay .005" >> /etc/chrony/chrony.conf
