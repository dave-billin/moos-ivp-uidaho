#!/bin/sh
##=============================================================================
## build.sh
##
## A shell script to build the moos-ivp-uidaho source tree for the Gumstix
## (TI OMAP3) CPU target
##
## History:
##  - Dave Billin: Created August 28, 2011
##
##=============================================================================
BUILD_DIR=build
CURRENT_DIR=$(pwd)

#-------------------------------------------------------------------
# Define some terminal colors for convenience
#-------------------------------------------------------------------

txtrst=$(tput sgr0)     # Reset colors and formatting for subsequent text
txtred=$(tput setaf 1)  # Start Red text
txtgrn=$(tput setaf 2)  # Start Green text
txtyel=$(tput setaf 3)  # Start Yellow text
txtblu=$(tput setaf 4)  # Start Blue text
txtvio=$(tput setaf 5)  # Start Purple text
txtcyn=$(tput setaf 6)  # Start Cyan text
txtwht=$(tput setaf 7)  # Start White text

txtbold=$(tput bold)  # Start bold text
txtul=$(tput smul)    # Start underlining text
txtulend=$(tput rmul) # Stop underlining text


#------------------------------------------------
# Verify that the OMAP3 toolchain is installed
#------------------------------------------------
if [ ! -x '/usr/local/angstrom/arm/bin/arm-angstrom-linux-gnueabi-gcc' ]; then
    echo "${txtred}"\
         "The OMAP3 C cross-compiler was not found at /usr/local/angstrom/arm/bin/"
    echo "${txtrst}${txtred}"\
         "A working cross-toolchain is required to build for the Gumstix OVERO."\
         "${txtrst}"
    echo ""
    exit 1;
fi

if [ ! -x '/usr/local/angstrom/arm/bin/arm-angstrom-linux-gnueabi-g++' ]; then
    echo "${txtred}"\
         "The OMAP3 C++ cross-compiler was not found at /usr/local/angstrom/arm/bin/"
    echo "${txtrst}${txtred}"\
         "A working cross-toolchain is required to build for the Gumstix OVERO."\
         "${txtrst}"
    echo ""
    exit 1;
fi

echo "${txtgrn}OMAP3 compilers detected.${txtrst}"


echo "${txtgrn}Building for the Gumstix OVERO...${txtrst}"

mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain-gumstix-omap3.cmake ..

make
cd ${CURRENT_DIR}


