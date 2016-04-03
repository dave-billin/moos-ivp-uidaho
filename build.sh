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

#-------------------------------------------------------------------
# Prints script usage then exits
#-------------------------------------------------------------------
function print_usage_and_exit ()
{
echo "
USAGE: ${_script_name} [OPTIONS...] [make arguments...]
OPTIONS:
   -h, --help   Print usage info and exit

   --build-tests
      Build unit tests

   --ivp-dir DIR
      Use DIR as the base directory of the IvP source tree when
      configuring CMake

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
cmake_args=

while test ${#} -gt 0
do
   OPT="${1}"
   OPTARG=
   [ ${#} -gt 1 ] && OPTARG="${2}"

   # DEBUG
   #echo "{ OPT=\"${OPT}\"  OPTARG=\"${OPTARG}\" }"

   case ${OPT} in
      --help|-h)
         print_usage_and_exit;;
     
      --build-tests)
         cmake_args+=" -DBUILD_UNIT_TESTS=yes";;

      --ivp-dir)
         [ -z "${OPTARG}" ] && { echo "--ivp-dir switch is missing required DIR argument" >&2; exit 1; }
         cmake_args+=" -DMOOSIVP_SOURCE_TREE_DIR=${OPTARG}"
         shift;;

      --purge|-p)
         purge_requested='yes';;

      --) shift
          cmd_args+=" ${@}"
          break;;

      *)  cmd_args+=" ${OPTARG}";;

   esac
   
   shift
done

# Implement purge
if [ 'yes' = "${purge_requested}" ]
then
   echo -e "Purging build directory...\n"
   [ -d "${_build_dir}" ] && (cd ${_build_dir}; make clean || true)
   rm -rf ${_build_dir} >/dev/null
   exit 0

else

   # Create the build directory if it doesn't exist
   mkdir -p ${_build_dir} 

   cd ${_build_dir}
   echo "Configuring CMake..."
   cmake ${cmake_args} ..

   echo -e "\nRunning 'make ${@}'"
   make ${cmd_args}
fi
