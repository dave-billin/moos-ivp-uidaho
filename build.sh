#!/bin/bash

##=============================================================================
## build.sh
##
## A shell script to build the moos-ivp-uidaho source tree
##
## History:
##  - Dave Billin: Created August 28, 2011
##  - M. Benjamin: Modified Sep 08, 2011
##=============================================================================

PWD=`pwd`
BUILD_DIR="build"
HELP="no"
CLEAN="no"

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


#-------------------------------------------------------------------
# A function that handles a request for help
#-------------------------------------------------------------------
function print_help ()
{
    printf "${txtbold}USAGE:  ${txtblu}build.sh ${txtrst}[SWITCHES]\n"
    printf "\n"
    printf "${txtul}SWITCHES:${txtulend}                           \n" 
    printf "  ${txtcyn}--clean, clean                    \n" 
    printf "  --help, -h${txtrst}                        \n"
    printf "\n"
    printf "${txtul}Notes:${txtulend}                              \n"
    printf " (1) All other command line args will be passed as args    \n"
    printf "     to \"make\" when it is eventually invoked.            \n"
    printf " (2) For example -k will continue making when/if a failure \n"
    printf "     is encountered in building one of the subdirectories. \n"
    printf " (3) For example -j2 will utilize a 2nd core in the build  \n"
    printf "     if your machine has two cores. -j4 etc for quad core. \n"
    printf "\n"

    exit 0
}



#-------------------------------------------------------------------
# A function that handles a request to clean the project
#-------------------------------------------------------------------
function do_clean ()
{
    cd ${BUILD_DIR}

    printf "${txtgrn}CLEANING...${txtrst}"

    # If a makefile exists, run its clean rule    
    if [ -e "./Makefile" ]; then
        make clean
    fi

    # Remove residual CMake files
    rm -rf CMakeFiles/ CMakeCache.txt Makefile src/ cmake_install.cmake
    rm -rf CPackConfig.cmake CPackSourceConfig.cmake Doxyfile
    printf "${txtgrn}done${txtrst}\n"
    cd ${PWD}

    exit 0;
}



#-------------------------------------------------------------------
# A function that completely blows away the build directory
#-------------------------------------------------------------------
function do_nuke ()
{
    printf "${txtred}Nuking the contents of ${BUILD_DIR}... (boom)${txtrst}\n"
    rm -rf ${BUILD_DIR};
}


#-------------------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------------------
case ${1} in
    -h) ;&
    --help)
        print_help;;

    clean) ;&
    --clean)
        do_clean;;

    nuke) ;&
    --nuke)
        do_nuke;
        exit 0;;

    rebuild) ;&
    --rebuild)
        do_nuke;
        shift 1;;

esac


#-------------------------------------------------------------------
# By default, configure and build in the ${BUILD_DIR} directory
#-------------------------------------------------------------------

# If no command line arguments were given, default to building all targets.
mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}

printf "${txtgrn}Configuring...${txtrst}\n"
cmake ..

printf "${txtgrn}Building...${txtrst}\n"
if [ -z $@ ]; then
    make 
else
    make $@
fi

cd ${PWD}

