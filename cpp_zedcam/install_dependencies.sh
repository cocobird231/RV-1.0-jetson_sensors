#!/usr/bin/bash

target_dir="$HOME/jetson_sensors"

# Check Internet Connection
printf "%s" "Internet connecting..."
while ! ping -w 1 -c 1 -n 168.95.1.1 &> /dev/null
do
    printf "%c" "."
done
printf "\n%s\n" "Internet connected."

# Check pwd
if [ "$PWD" == "$target_dir" ]
then
    echo "In $target_dir"
else
    if ls $target_dir &> /dev/null
    then
        cd $target_dir
        echo "Change directory: $PWD"
    else
        echo "jetson_sensors path error. Please copy jetson_sensors directory under $HOME"
        exit 1
    fi
fi
# pwd in ~/jetson_sensors

# Check Jetpack version
jetson_vers=$(sudo apt-cache show nvidia-jetpack | grep Version | grep -Po '(\d+\.\d+)(\.\d+)?')
jetson_ver=$(echo $jetson_vers | grep -Po '^(\d+\.\d+)(\.\d+)?')
jetson_main_ver=$(echo $jetson_ver | grep -Po '^(\d+\.\d+)')

# Remove old zed-install.run and grab new one from https://www.stereolabs.com/developers/release/
sudo apt install zstd
rm -rf zed-install.run
if [ "$jetson_main_ver" == "4.6" ]
then
    wget -O zed-install.run https://download.stereolabs.com/zedsdk/4.0/l4t32.7/jetsons
elif [ "$jetson_main_ver" == "5.0" ]
then
    wget -O zed-install.run https://download.stereolabs.com/zedsdk/4.0/l4t35.1/jetsons
elif [ "$jetson_main_ver" == "5.1" ]
then
    if [ "$jetson_ver" == "5.1.1" ]
    then
        wget -O zed-install.run https://download.stereolabs.com/zedsdk/4.0/l4t35.3/jetsons
    else
        wget -O zed-install.run https://download.stereolabs.com/zedsdk/4.0/l4t35.2/jetsons
    fi
else
    echo "Jetpack version not supported."
    exit 1
fi

# Check zed-install.run file
if ls zed-install.run &> /dev/null
then
    echo "Start ZED SDK installation..."
    sudo chmod a+x zed-install.run
    ./zed-install.run -- silent skip_od_module skip_python skip_hub skip_tools
else
    echo "zed-install.run not found. Exiting..."
    exit 1
fi

# Install OpenCV
echo "Start OpenCV installation..."
sudo apt install libopencv-dev