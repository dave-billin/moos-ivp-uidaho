#!/bin/bash
#==============================================================================
# DESCRIPTION:
#   This script executes make clean and recursively deletes all CMake-generated 
#   files and directories in the build tree.  This simplifies the task of 
#   re-generating a CMake configuration if an in-source build is being 
#   performed.
#
# USAGE:    See content printed by the print_usage function below
# 
# AUTHOR:   Dave Billin <david.billin@vandals.uidaho.edu>
#==============================================================================

# Canonical values for readability
TRUE=0
FALSE=1

# terminal formatting strings
txtrst=$(tput sgr0)     # Reset colors for subsequent text
txtred=$(tput setaf 1)  # Start Red text
txtgrn=$(tput setaf 2)  # Start Green text
txtcyn=$(tput setaf 6)  # Start Cyan text



#========================
# Default values
#========================
START_PATH=$(dirname ${0})
BYPASS_CONFIRMATION=${FALSE}
TEST_ONLY=${FALSE}


#==============================================================================
# A function to print usage info
function print_usage ()
{
echo "
  USAGE: CleanProject [OPTIONS] [START_PATH]

     OPTIONS
        --cwd           Makes START_PATH relative to the current working
                        directory rather than the directory CleanProject.sh 
                        resides in

        -h, --help      Print usage info and exit
        -y, --yes       Bypass confirmation prompt
        -t, --test      Test clean - prints the files that would be deleted
                        without actually deleting them

     START_DIR
        Subdirectory to recursively clean.  If not given, recursive cleaning 
        will begin in the current directory.
"
exit ${TRUE}
}
#==============================================================================


#==============================================================================
#==============================================================================
#   -- SCRIPT JOBS START HERE --
#==============================================================================
#==============================================================================


#========================
# Parse script arguments
#========================
while [[ $# > 0 ]]; do
    case ${1} in
        --cwd)
            START_PATH=$(pwd);;

        -h) ;&
        --help)
            print_usage ;;

        -t) ;&
        --test)
            TEST_ONLY=${TRUE};;

        -y) ;&
        --yes)
            BYPASS_CONFIRMATION=${TRUE};;

        -*)
            echo "Unrecognized option: ${1}"
            exit ${FALSE};;

        *)
            START_PATH="${START_PATH}/${1}";;
    esac

    shift 1
done


#===========================
# Validate start directory
#===========================
test ! -d ${START_PATH} && echo "CleanProject target path '${START_PATH}' does not exist!" && exit ${FALSE}


#===========================
# Issue a nag prompt
#===========================
# Implement nag bypass
if [ ${BYPASS_CONFIRMATION} -eq ${TRUE} ] || [ ${TEST_ONLY} -eq ${TRUE} ]; then
    CONFIRM=${TRUE}
else
    echo "This will delete all CMake-generated files in ${START_PATH}."
    echo -n "Are you sure (Y/n)? "

    while :
    do
        read CONFIRM
        case ${CONFIRM} in
            Y) CONFIRM=${TRUE}; break;;
            n) CONFIRM=${FALSE}; break;;
            *) CONFIRM=""
               echo -n "Are you sure (Y/n)? ";;
        esac
    done
fi


# DEBUG
#echo "START_PATH=${START_PATH}"
#echo "BYPASS_CONFIRMATION=${BYPASS_CONFIRMATION}"
#echo "CONFIRM=${CONFIRM}"
#echo "VERBOSE = ${VERBOSE}"
#exit ${TRUE}
# END DEBUG

test !${CONFIRM} == ${TRUE} && exit ${TRUE}

PRINT_COMMAND="-print"


RM_COMMAND="-exec rm -rf {} \;"
if [ ${TEST_ONLY} -eq ${TRUE} ]; then
    RM_COMMAND=""
    echo "
    ** TEST CLEAN - NO FILES WILL BE DELETED **"
fi


echo "${txtcyn}
Deleting CMakeFiles directories...${txtred}"

eval "find ${START_PATH} -not -iwholename '*.svn' -iname 'CMakeFiles' -type d -prune ${PRINT_COMMAND} ${RM_COMMAND}"

echo "${txtcyn}
Deleting CMakeCache.txt files...${txtred}"
eval "find ${START_PATH} -not -iwholename '*.svn' -iname 'CMakeCache.txt' -type f  ${PRINT_COMMAND} ${RM_COMMAND}"

echo "${txtcyn}
Deleting cmake_install files...${txtred}"
eval "find ${START_PATH} -not -iwholename '*.svn' -iname 'cmake_install.cmake' -type f  ${PRINT_COMMAND} ${RM_COMMAND}"

echo "${txtcyn}
Deleting Makefiles...${txtred}"
eval "find ${START_PATH} -not -iwholename '*.svn' -iname 'Makefile' -type f  ${PRINT_COMMAND} ${RM_COMMAND}"

echo "${txtcyn}
Deleting CPack files...${txtred}"
eval "find ${START_PATH} -not -iwholename '*.svn' -iname 'CPack*.cmake' -type f  ${PRINT_COMMAND} ${RM_COMMAND}"

echo "${txtcyn}
Deleting CTest files...${txtred}"
eval "find ${START_PATH} -not -iwholename '*.svn' -iname 'CTest*.cmake' -type f  ${PRINT_COMMAND} ${RM_COMMAND}"

#=========================
# Print exit message
#=========================
if [ ${TEST_ONLY} -eq ${TRUE} ]; then
    echo "${txtrst}
    ** TEST CLEAN - NO FILES HAVE BEEN DELETED **
    "
else
    echo "
    ${txtgrn}(sweep, sweep...) This project is clean!${txtrst}
    "
    
fi

exit ${TRUE}

