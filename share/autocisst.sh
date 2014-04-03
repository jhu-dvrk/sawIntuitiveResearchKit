#!/usr/bin/env bash

# Author: Zihan Chen
# Date: 2013-05-01
# EmaiL: zihan.chen@jhu.edu

# create source & build dir
mkdir build

# git clone source
git clone https://github.com/jhu-cisst/cisst-saw.git --recursive

# download cisstNetlib based on system architecture
if [ $(uname -m) == "x86_64" ]; then
    # 64-bit version
    wget http://unittest.lcsr.jhu.edu/cisst/downloads/cisstNetlib/current/cisstNetlib-Linux-x86_64-2007-04-09.tar.gz
else
    # 32-bit version
    wget http://unittest.lcsr.jhu.edu/cisst/downloads/cisstNetlib/current/cisstNetlib-Linux-i686-2007-04-09.tar.gz
fi

# extract & rename cisstNetlib & delete tarball
tar xfz cisstNetlib-Linux-*-2007-04-09.tar.gz
mv cisstNetlib-Linux-*-2007-04-09 cisstNetlib-Linux
rm cisstNetlib-Linux-*-2007-04-09.tar.gz

# to build & cmake
cd build
# download cisst initial-cache
wget https://raw.githubusercontent.com/jhu-dvrk/sawIntuitiveResearchKit/master/share/dvrk.cisst.initial.cmake

# cmake
cmake -C dvrk.cisst.initial.cmake ../cisst-saw
# build
make -j4 -l

# source cisstvar.sh
source ./cisst/cisstvars.sh
