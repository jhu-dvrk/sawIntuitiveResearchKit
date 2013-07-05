#!/usr/bin/env bash

# Author: Zihan Chen
# Date: 2013-05-01
# EmaiL: zihan.chen@jhu.edu

# create source & build dir
mkdir source
mkdir build

# svn checkout source
svn co https://svn.lcsr.jhu.edu/cisst/trunk ./source


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
wget https://trac.lcsr.jhu.edu/cisst/raw-attachment/wiki/sawIntuitiveResearchKitTutorial/irk.cisst.initial.cmake
# cmake
cmake -C irk.cisst.initial.cmake ../source
# build
make -j8 -l


# source cisstvar.sh
source ./cisstvars.sh


# OPTIONAL

# load cisstvar.sh by default
echo "if [ -f ~/dev/cisst/build/cisstvars.sh ]; then" >> ~/.bashrc
echo "    . ~/dev/cisst/build/cisstvars.sh" >> ~/.bashrc
echo "fi" >> ~/.bashrc


