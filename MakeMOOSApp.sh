#!/bin/bash
#==============================================================================
# File: MakeMOOSApp.sh
#
# Description:
#   A shell script that generates a new boilerplate MOOS application and adds 
#   it to the moos-ivp-uidaho source tree
#
# Author: Dave Billin
#
# History:
#   12-20-2011 - Initial creation (the big shebang!)
#
# USAGE:    MakeMOOSApp [options]
#
#   OPTIONS:
#       -a AUTHOR       Specify the application's author
#       -d              Path where the new app will be stored (default is ./src)
#       -e ADDRESS      Specify the author's e-mail address
#       -h, -?          Print usage info
#       -n NAME         Specify the name of the new application
#
#==============================================================================
SCRIPT_PATH=$(dirname ${0})
APPFACTORYPATH=${SCRIPT_PATH}/config/AppFactory # Directory containing AppFactory templates
TARGETPATH=${SCRIPT_PATH}/projects  # Directory where created app should be placed
TARGETCMAKECONFIG=${SCRIPT_PATH}/projects/CMakeLists.txt


#------------------------------------
# Variables for stdio formatting
#------------------------------------
txtrst=$(tput sgr0)     # Reset colors and formatting to defaults
txtred=$(tput setaf 1)  # Start Red text
txtgrn=$(tput setaf 2)  # Start Green text
txtyel=$(tput setaf 3)  # Start Yellow text

txtbold=$(tput bold)  # Start bold text
txtul=$(tput smul)    # Start underlining text
txtulend=$(tput rmul) # Stop underlining text



#------------------------------------------------------------------------------
# A function to print the script's usage info
PrintUsageInfo ()
{
echo "
${txtbold}USAGE:${txtrst}  MakeMOOSApp.sh [options] [${txtbold}APP_NAME${txtrst}]

  ${txtul}OPTIONS${txtulend}:
    -a,--author AUTHOR     Specify the application's author
    -d,--dir PATH          Path where the new app will be stored
                           (default is ./src)
    -e,--email ADDRESS     Specify the author's e-mail address
    -h,--help              Print this usage info
${txtbold}
  APP_NAME${txtrst}        Name of the MOOS app to create
"
}
#------------------------------------------------------------------------------



#---------------------
# PARSE OPTIONS
#---------------------
APP_AUTHOR=""
APP_EMAIL=""
TARGETPATH=${SCRIPT_PATH}/projects  # Directory where created app should be placed

# Recall: a colon in the getopts list signals a required argument for the
# associated switch
while [ ! -z "${1}" ]; do
case ${1} in
    -a) ;&
    --a) APP_AUTHOR="${2}"; shift 1;;

    -d) ;&
    --dir) TARGETPATH="${2}"; shift 1;;

    -e) ;&
    --email) APP_EMAIL="${2}"; shift 1;;

    -h) ;&
    --help) PrintUsageInfo; exit 0;;

    -*) printf "${txtred}Illegal option: ${OP}\n${txtrst}"; exit -1;;

    *) APP_NAME=${1};;
esac

shift 1
done

    
#-------------------------------------
# Get the name of the new app
#-------------------------------------
while [ -z "${APP_NAME}" ];
do
    printf "Enter the application name: "
    read APP_NAME;
done

APP_NAME_CAPS=$(echo "${APP_NAME}" | tr a-z A-Z )
APP_PATH="${TARGETPATH}/${APP_NAME}"


#-------------------------------------
# Get the app author
#-------------------------------------
if [ -z "${APP_AUTHOR}" ]; then
    printf "Application author (ENTER to leave blank): "
    read APP_AUTHOR;

    # Fill in a default author if none is given
    if [ -z "${APP_AUTHOR}" ]; then
        APP_AUTHOR="University of Idaho"
    fi
fi


#-------------------------------------
# Get the author's e-mail address
#-------------------------------------
if [ -z "${APP_EMAIL}" ]; then
    printf "Author e-mail address (ENTER to leave blank): "
    read APP_EMAIL;
fi


SRC_DIR=${APP_PATH}/src
INCLUDE_DIR=${APP_PATH}/include
CONFIG_DIR=${APP_PATH}/config
DOCS_DIR=${APP_PATH}/docs

#-----------------------------------------------
# CREATE A DIRECTORY STRUCTURE FOR THE NEW APP
#-----------------------------------------------
if [ -d ${APP_PATH} ]; then
    printf "${txtyel}${APP_PATH} already exists.\n"
    printf "Should it be overwritten (y/n)? ${txtrst}"
    read REPLY
    if [ "$REPLY" == "n" -o "$REPLY" == "N" ]; then
        printf "\n${txtred}Aborted creation of ${APP_NAME}.\n\n${txtrst}"
        exit 0
    else
        if [ ! -z "$(which svn)" ]; then
            svn rm --force ${APP_PATH}
        else
            rm -rf ${APP_PATH}
        fi
    fi
fi

printf "${txtgrn}Setting up project directory...\n${txtrst}"

mkdir -p ${SRC_DIR}
mkdir -p ${INCLUDE_DIR}
mkdir -p ${CONFIG_DIR}
mkdir -p ${DOCS_DIR}
cp ${APPFACTORYPATH}/config/* ${CONFIG_DIR}
#cp ${APPFACTORYPATH}/docs/* ${DOCS_DIR}



#-----------------------------------------------
# COPY IN TEMPLATE FILES 
#-----------------------------------------------
cp ${APPFACTORYPATH}/main.cpp ${SRC_DIR}/main.cpp
cp ${APPFACTORYPATH}/AppObjectTemplate.cpp ${SRC_DIR}/${APP_NAME}.cpp
cp ${APPFACTORYPATH}/AppObjectTemplate.h ${INCLUDE_DIR}/${APP_NAME}.h
cp ${APPFACTORYPATH}/CMakeLists.txt ${APP_PATH}/CMakeLists.txt
cp ${APPFACTORYPATH}/MissionFile.moos ${APP_PATH}/${APP_NAME}.moos
cp ${APPFACTORYPATH}/README.txt ${APP_PATH}/README.txt



#------------------------------------------
# PROCESS MACROS IN TEMPLATE FILES
#------------------------------------------
printf "${txtgrn}Processing template files...\n${txtrst}"

# Make a list of files to process
FILES_TO_PROCESS=$(find ${APP_PATH} -maxdepth 2 -type f)

for F in ${FILES_TO_PROCESS}
do
    sed -i "s/MOOSAPPFACTORY_NAME/${APP_NAME}/g
          s/MOOSAPPFACTORY_CAPITALNAME/${APP_NAME_CAPS}/g
          s/MOOSAPPFACTORY_AUTHOR/${APP_AUTHOR}/g
          s/MOOSAPPFACTORY_YEAR/$(date +'%C%y')/g
          s/MOOSAPPFACTORY_EMAIL/${APP_EMAIL}/g" ${F}
done



#----------------------------------------------------
# Add the new app directory to the moos-ivp-uidaho
# CMake configuration
#----------------------------------------------------
# Build a string to append to the moos-ivp-uidaho CMake configuration
CMAKE_ENTRY=$( echo "


#===========================
# ${APP_NAME}
#===========================
option( BUILD_APP_${APP_NAME_CAPS} \"Build the ${APP_NAME} Utility\" ON )
if (BUILD_APP_${APP_NAME_CAPS})
add_subdirectory(${APP_NAME})
endif()" )

CMAKE_DUPLICATE=$(grep "add_subdirectory(${APP_NAME})" ${SCRIPT_PATH}/projects/CMakeLists.txt)

# Don't create a duplicate entry in the moos-ivp-uidaho 
# CMake configuration if one already exists
if [ -z "$CMAKE_DUPLICATE" ]; then
    printf "${txtgrn}Adding ${APP_NAME} to projects/CMakeLists.txt...${txtrst}\n"
    echo "${CMAKE_ENTRY}" >> ${SCRIPT_PATH}/projects/CMakeLists.txt
else
    printf "${txtgrn}NOTE: An entry for ${APP_NAME} already exists in "
    printf "the moos-ivp-uidaho CMake configuration.${txtrst}\n"
fi

#----------------------------------
# Add the project directory to
# source control
#----------------------------------
SVN=$(which svn)
if [ ! -z "${SVN}" ]; then
    echo ""
    echo "${txtgrn}Adding ${APP_NAME} to source control${txtrst}"
    svn add ${APP_PATH} 
    echo ""
fi

#----------------------------------
# All done!  Print exit comments
#----------------------------------
printf "\n${txtgrn}"
printf "The ${APP_NAME} application was added to the moos-ivp-uidaho CMake "
printf "configuration:\n"
printf "${txtyel}${SCRIPT_PATH}/projects/CMakeLists.txt${txtgrn}\n"
printf "You may need to run CMake again for changes to take effect.\n\n"
printf "${txtrst}"
exit 0

