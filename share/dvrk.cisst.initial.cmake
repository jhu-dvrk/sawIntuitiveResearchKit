########################
# Initial cache settings for cisst for sawIntuitiveResearchKit
# run cmake with:
# cmake -C 
########################

# Author: Zihan Chen
# Date: 2013-05-01
# EmaiL: zihan.chen@jhu.edu

# Brief: this file is written for people who are not familiar with cisst

#Build with cisst netlib (for numerical)
set(CISST_HAS_CISSTNETLIB ON CACHE BOOL "" )
set(CISSTNETLIB_DIR ${CMAKE_BINARY_DIR}/../cisstNetlib-Linux/ CACHE PATH "")

# -------- CISST -----------
# XML parsing
set(CISST_cisstCommonXML ON CACHE BOOL "" )
# JSON 
set(CISST_HAS_JSON ON CACHE BOOL "" )


# -------- SAW -----------
# sawRobotIO1394
set(SAW_sawRobotIO1394 ON CACHE BOOL "")
# sawController (PID / GC / Teleoperation)
set(SAW_sawControllers ON CACHE BOOL "")
# sawTextToSpeech (For Error Messaging)
set(SAW_sawTextToSpeech ON CACHE BOOL "")
# sawIntuitiveResearchKit
set(SAW_sawIntuitiveResearchKit ON CACHE BOOL "")

