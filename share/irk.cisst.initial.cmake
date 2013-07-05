########################
# Initial cache settings for cisst for sawIntuitiveResearchKit
# run cmake with:
# cmake -C 
########################

# Author: Zihan Chen
# Date: 2013-05-01
# EmaiL: zihan.chen@jhu.edu

# Brief: this file is written for people who are not familiar with
# cisst and only want to use ROS interface 

#Build with cisst netlib (for numerical)
set(CISST_HAS_CISSTNETLIB ON CACHE BOOL "" )
set(CISSTNETLIB_DIR ${CMAKE_BINARY_DIR}/../cisstNetlib-Linux/ CACHE PATH "")
#set(CISSTNETLIB_DOWNLOAD_NOW ON CACHE BOOL "")
#set(CISSTNETLIB_DOWNLOAD_ARCHITECTURE x86_64 CACHE STRING "")

# -------- CISST -----------
set(CISST_cisstCommonXML ON CACHE BOOL "" )


# -------- SAW -----------
# build saw (Surgical Assistant Workstation)
set(CISST_BUILD_SAW ON CACHE BOOL "")
# sawRobotIO1394
set(SAW_RobotIO1394 ON CACHE BOOL "")
# sawController (PID / GC / Teleoperation)
set(SAW_Controllers ON CACHE BOOL "")
# sawTextToSpeech (For Error Messaging)
set(SAW_TextToSpeech ON CACHE BOOL "")
# sawIntuitiveResearchKit
set(SAW_IntuitiveResearchKit ON CACHE BOOL "")

