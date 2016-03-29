#!/bin/bash
##=============================================================================
## build.sh
##
## A shell script to build the moos-ivp-uidaho source tree
##
## History:
##  - Dave Billin: Created August 28, 2011
##=============================================================================

set -o errexit   # Exit on first error
set -o nounset   # Disallow unset variables
#set -o xtrace   # Trace commands as they are executed

_script_name="$(basename ${0})"
_script_dir="$(readlink -f $(dirname ${0}))"

_build_dir="${_script_dir}/build"

PWD=`pwd`
_build_dir="build"
HELP="no"
CLEAN="no"

#-------------------------------------------------------------------
# Prints script usage then exits
#-------------------------------------------------------------------
function print_usage_and_exit ()
{
echo "
USAGE: ${_script_name} [OPTIONS...] [make arguments...]
OPTIONS:
   -h, --help   Print usage info and exit
   -p, --purge  Delete all existing build artifacts

make arguments:
   Any command line arguments not included in OPTIONS are passed as
   parameters to make
"
   exit 0
}


#-------------------------------------------------------------------
#  Parse command line arguments
#-------------------------------------------------------------------
purge_requested='no'
cmd_args="-j12"

for ARGI
do
   case ${ARGI} in
      --help|-h)
         print_usage_and_exit;;

      --purge|-p)
         purge_requested='yes';;

      *)  cmd_args+=" ${ARGI}";;
   esac
done


# Implement purge
if [ 'yes' = "${purge_requested}" ]
then
   [ -d "${_build_dir}" ] && (cd ${_build_dir} && make clean || true)
   rm -rf ${_build_dir}
   exit 0

else

   # Create the build directory if it doesn't exist
   mkdir -p ${_build_dir} 

   cd ${_build_dir}
   echo "Configuring CMake..."
   cmake ..

   echo -e "\nRunning 'make ${@}'"
   make ${cmd_args}
fi
