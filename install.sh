#!/bin/bash

# Bash script to install supporting software for the Robotics Cape
# tested on follwing beagleboard.org Debian releases 
# 2015-11-12
# Modified by Kiran Kumar Lekkala and will be used for building and installing bb_blue api on Beaglebone Black

INSTALL_DIR="/root"
BOOTSCRIPT="Auto_Run_Script.sh"
OVERLAY="BBB_BASE-00A0"
CAPENAME="BB_BLUE"
KERNEL="$(uname -r)"
DEBIAN="$(cat /etc/debian_version)"

CONFIG_DIR="/etc/bb_blue_api"
AUTO_RUN_DIR="/root/Auto_Run_Programs"

echo " "


echo " "


# make sure the user is root
if [ `whoami` != 'root' ]; then
	echo "You must be root to install this"
	exit
fi

	echo "please use one of the following Debian images"
	echo "2016-05-01"
	exit
fi 
echo "using linux kernel $KERNEL"

# warn the user if using a 'ti' kernel instead of a 'bone' kernel
if uname -r | grep -q "ti"; then
	echo "WARNING: the 'ti' kernels do not necessarily"
	echo "support the uio-pruss PRU driver."
	echo "we suggest using a 'bone' kernel"
fi

#check that the uio-pruss driver is available
if modprobe -n pru_rproc | grep -q "not found"; then
	echo "ERROR: uio-pruss driver missing."
	echo "We suggest using the latest Debian Wheezy image."
	echo "The Debian Jessie image does not yet support the PRU."
	exit
fi

#check dependencies
if [ ! -f /usr/bin/make ]; then
	echo " "
    echo "error: dependency 'make' not installed"
	echo "use apt-get install build-essentials"
	echo "OR, if you are using a console-only image:"
	echo "bash upgrade_console_only_image.sh"
	exit
fi

if [ ! -f /usr/bin/gcc ]; then
	echo " "
    echo "error: dependency 'gcc' not installed"
	echo "use apt-get install build-essentials"
	echo "OR, if you are using a console-only image:"
	echo "bash upgrade_console_only_image.sh"
	exit
fi

if [ ! -f /usr/lib/libprussdrv.so ]; then
	if [ ! -f /usr/lib/local/libprussdrv.so ]; then
		echo " "
		echo "error: libprussdrv missing"
		exit
	fi
fi

# make sure the user really wants to install
echo ""
echo "This script will install all Beaglebone Blue supporting software."
read -r -p "Continue? [y/n] " response
case $response in
    [yY]) echo " " ;;
    *) echo "cancelled"; exit;;
esac
echo " "


# touch everything since the BBB clock is probably wrong
find . -exec touch {} \;

echo "Installing Device Tree Overlay"
# newer images ship with dtc compiler
dtc -O dtb -o /lib/firmware/$OVERLAY.dtbo -b 0 -@ install_files/2016-05-01/$OVERLAY.dts



echo "Installing PRU Binaries"
cp install_files/pru/am335x-pru0-fw /lib/firmware

echo "Rebooting pru-core 0"
echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/unbind 2>/dev/null
echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/bind


echo "Installing Supporting Libraries"
cd libraries
make clean > /dev/null
make install > /dev/null
make clean
cd ../


echo "Installing examples, this will take a minute."
find examples/ -exec touch {} \;
cd examples
make clean > /dev/null
make install > /dev/null
make clean > /dev/null
cd ../

# make sure config diectory exists
if [ ! -a "$CONFIG_DIR" ]; then
	mkdir $CONFIG_DIR
fi

# make sure Auto Run directory exists
if [ ! -a "$AUTO_RUN_DIR" ]; then
	echo "creating directory " $AUTO_RUN_DIR
	mkdir $AUTO_RUN_DIR
fi

# set up a script to run things on boot
echo "Enabling Boot Script"
cp install_files/$BOOTSCRIPT /etc/init.d/
chmod 755 /etc/init.d/$BOOTSCRIPT
update-rc.d $BOOTSCRIPT defaults 

echo " "
echo "Which program should run on boot?"
echo "This will overwrite any program in " $AUTO_RUN_DIR
echo "Select 'existing' to keep current configuration."
echo "type 1-5 then enter"
select bfn in "blink" "balance" "none" "existing"; do
    case $bfn in
		blink ) PROG="blink"; break;;
        balance ) PROG="balance"; break;;
		none ) PROG="none"; break;;
		existing ) PROG="existing"; break;;
    esac
done

# if user didn't select existing, delete everything in auto run folder
# and replace with new program
if [ "$PROG" != "existing" ]; then
	rm -f $AUTO_RUN_DIR/*
	if [ "$PROG" != "none" ]; then
		cp /usr/bin/$PROG $AUTO_RUN_DIR/
	fi
fi

# all done
echo " "
echo "Robotics Cape Configured and Installed"
echo "Reboot to complete installation."
echo "After Rebooting we suggest running calibrate_gyro"
echo " " 
