#==============================================================================
# CMakeLists.txt
# CMake configuration file for the moos-ivp-uidaho project's "src" folder
#
# Created 8-2011 by Dave Billin
#==============================================================================


#===============================================
# PROJECT SOURCE CODE
#===============================================
# Here we specify subdirectories containing source packages for individual
# project components.  CMake will recurse into each of them in the order which
# they are given here and process each folder's respective CMakeLists.txt 
# configuration files.



#===========================
# lib_YellowSubUtils
#===========================
option( BUILD_LIB_YELLOWSUBUTILS "Build the YellowSubUtils static library" ON )
if (BUILD_LIB_YELLOWSUBUTILS)
    add_subdirectory(lib_YellowSubUtils)
    include_directories( ${YELLOWSUBUTILS_INCLUDE_DIRECTORIES} )
endif()


#===========================
# lib_BunnySock
#===========================
option( BUILD_LIB_BUNNYSOCK "Build BunnySock communications static library" ON )
if (BUILD_LIB_BUNNYSOCK)
    add_subdirectory(lib_BunnySock)
endif()


#===========================
# lib_parapet
#===========================
option( BUILD_LIB_PARAPET "Build parapet communications static library" ON )
if (BUILD_LIB_PARAPET)
    add_subdirectory(lib_parapet)
    include_directories( ${PARAPET_INCLUDE_DIRS} )
endif()


#===========================
# YellowSubNav library
#===========================
option( BUILD_LIB_YELLOWSUBNAV "Build YellowSubNav navigation library" ON )
if (BUILD_LIB_YELLOWSUBNAV)
    add_subdirectory(lib_Newmat10)
    include_directories( ${NEWMAT10_INCLUDE_DIRECTORIES} )

    add_subdirectory(lib_YellowSubNav)
    include_directories( ${YELLOWSUBNAV_INCLUDE_DIRECTORIES} )
endif()


#===========================
# BHV_ReverseDive
#===========================
option(BUILD_BHV_REVERSEDIVE "Build the ReverseDive IvP Helm behavior" ON )
if (BUILD_BHV_REVERSEDIVE)
    add_subdirectory(BHV_ReverseDive)
endif()



#===========================
# iArchangelIMU
#===========================
option(BUILD_APP_IARCHANGELIMU "Build iArchangelIMU MOOS instrument" ON )
if (BUILD_APP_IARCHANGELIMU)
    add_subdirectory(iArchangelIMU)
endif()

#===========================
# iBSDuC
#===========================
option( BUILD_APP_IBSDUC "Build the iBSDuC Utility" ON )
if (BUILD_APP_IBSDUC)
add_subdirectory(iBSDuC)
endif()

#===========================
# iGPSd
#===========================
option( BUILD_APP_IGPSD "Build the iGPSd Utility" ON )
if (BUILD_APP_IGPSD)
add_subdirectory(iGPSd)
endif()

#===========================
# iRevolutionCompass
#===========================
option( BUILD_APP_IREVOLUTIONCOMPASS "Build the iRevolutionCompass Utility" ON )
if (BUILD_APP_IREVOLUTIONCOMPASS)
add_subdirectory(iRevolutionCompass)
endif()


#===========================
# iScotty
#===========================
option(BUILD_APP_ISCOTTY "Build the iScotty MOOS instrument (DEPRECATED)" ON )
if (BUILD_APP_ISCOTTY)
    add_subdirectory(iScotty)
endif()


#===========================
# iSpock
#===========================
option(BUILD_APP_ISPOCK "Build the iSpock MOOS instrument (DEPRECATED)" ON )
if (BUILD_APP_ISPOCK)
    add_subdirectory(iSpock)
endif()


##===========================
## iWhoiMicroModem
##===========================
option(BUILD_APP_IWHOIMICROMODEM "Build the iWhoiMicroModem MOOS instrument" ON )
if (BUILD_APP_IWHOIMICROMODEM)
    add_subdirectory(iWhoiMicroModem)
endif()


#===========================
# iXStreamRadioModem
#===========================
option(BUILD_APP_IXSTREAMRADIOMODEM "Build the iXStreamRadioModem MOOS instrument" ON )
if (BUILD_APP_IXSTREAMRADIOMODEM)
    add_subdirectory(iXStreamRadioModem)
endif()


#===========================
# iYellowSubDAQ
#===========================
option(BUILD_APP_IYELLOWSUBDAQ "Build the iYellowSubDAQ MOOS instrument" ON )
if (BUILD_APP_IYELLOWSUBDAQ)
    add_subdirectory(iYellowSubDAQ)
endif()


##===========================
## pAcousticCommsAgent
##===========================
#option( BUILD_APP_PACOUSTICOMSSAGENT "Build the pAcousticCommsAgent MOOS app" ON )
#if (BUILD_APP_PACOUSTICOMSSAGENT)
#    add_subdirectory(pAcousticCommsAgent)
#endif()
#
#
##===========================
## pFakeModemReport
##===========================
#option( BUILD_APP_PFAKEMODEMREPORT "Build the pFakeModemReport MOOS app" ON )
#if (BUILD_APP_PFAKEMODEMREPORT)
#    add_subdirectory(pFakeModemReport)
#endif()
#
#
##===========================
## pMissionMonitor
##===========================
#option( BUILD_APP_PMISSIONMONITOR "Build the pMissionMonitor MOOS app" ON )
#if (BUILD_APP_PMISSIONMONITOR)
#    add_subdirectory(pMissionMonitor)
#endif()
#
#
##===========================
## pVehicleEKF
##===========================
#option( BUILD_APP_PVEHICLEEKF "Build the pVehicleEKF MOOS app" ON )
#if (BUILD_APP_PVEHICLEEKF)
#    add_subdirectory(pVehicleEKF)
#endif()
#
#
##===========================
## pVehicleID
##===========================
#option( BUILD_APP_PVEHICLEID "Build the pVehicleID MOOS app" ON )
#if (BUILD_APP_PVEHICLEID)
#    add_subdirectory(pVehicleID)
#endif()
#
#
#
#
##===========================
## uGeodesy
##===========================
#option( BUILD_UTIL_UGEODESY "Build the uGeodesy Utility" ON )
#if (BUILD_UTIL_UGEODESY)
#    add_subdirectory(uGeodesy)
#endif()
#
#
#
#

