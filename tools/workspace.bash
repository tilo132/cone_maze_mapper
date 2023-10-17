#!/bin/bash
unset opt OPTARG OPTIND
unset help
unset kill_server
unset domain_id
unset source_ros
unset super_client
unset username
unset ip 
unset port
ws_path=$( dirname $(dirname "${BASH_SOURCE[0]}"))

ip=$(cat $ws_path/tools/super_client_configuration_file.xml | grep address | sed 's|[[:space:]]*<[^>]*>||g')
port=$(cat $ws_path/tools/super_client_configuration_file.xml | grep port | sed 's|[[:space:]]*<[^>]*>||g')
while getopts hkd:rsp: flag
do
    case "${flag}" in
    	h) help="help";;
        k) kill_server="kill";;
        d) domain_id=${OPTARG};;
        r) source_ros="source";;
        s) super_client="super";;
        p) username=${OPTARG};;
        
    esac
done

if ! [ -z ${help+x}]
then
	echo ""
	echo "         COMMAND       : DESCRIPTION"
	echo "-d ROS_DOMAIN_ID       : sets ros2 domain id"
	echo "-k kill                : kills the discovery server"
	echo "-r source ros          : sources this workspace and ros2"
	echo "-s super client        : loads the super_client_configuration_file and makes this shell a super client"
	echo "-p grant folder access : grants the specified user rwx permissions on the entire workspace. Needs sudo."
	echo "-h help                : displays this page"
	echo ""
	exit
fi

if ! [ -z ${source_ros+x} ]
then
	source /opt/ros/humble/setup.bash
	source $ws_path/install/setup.bash
	echo "Sourced ros"
fi


if ! [ -z "$username" ]
then
	sudo setfacl -R -m u:$username:rwx $ws_path
fi

if [ -z ${kill_server+x} ]
then
	export ROS_DISCOVERY_SERVER="[${ip}:${port};192.168.3.2:11812]"
	echo "Enabled Discovery Server: ${ROS_DISCOVERY_SERVER}"
	if [ -z ${super_client+x} ]
	then
		unset FASTRTPS_DEFAULT_PROFILES_FILE
		echo "This shell is lame and not super."
	else
		export FASTRTPS_DEFAULT_PROFILES_FILE=$ws_path/tools/super_client_configuration_file.xml
		echo "Runnging as SuperConfigurationClient!!!!!!!!"
	fi
	ros2 daemon stop
	ros2 daemon start
else
	unset ROS_DISCOVERY_SERVER
	unset FASTRTPS_DEFAULT_PROFILES_FILE
	ros2 daemon stop
	ros2 daemon start
	echo "Killed Disovery Server"
fi

if [ -z "$domain_id" ]
then
	export ROS_DOMAIN_ID=0
else
	export ROS_DOMAIN_ID=$domain_id
fi
echo "Set ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
