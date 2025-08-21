# CCAV-T
Cooperative &amp; Connected AV Testing Application

2. 환경 변수 설정

ROS_IP, ROS_MASTER_URI, ROS_HOSTNAME 3개의 환경변수를 설정해줘야한다.



# Primary

export ROS_IP={IP_of_P}

export ROS_MASTER_URI=http://$ROS_IP:11311

export ROS_HOSTNAME=$ROS_IP



# Secondary

export ROS_MASTER_URI=http://{IP_of_P}:11311

export ROS_HOSTNAME={IP_of_S}