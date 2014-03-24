#!/bin/sh

# Copyright (c) 2013 Regents of the SIGNET lab, University of Padova.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the University of Padova (SIGNET lab) nor the 
#    names of its contributors may be used to endorse or promote products 
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

color1='\033[0;34m'
color2='\033[1;31m'
nc='\033[0m'

info1()
{
    echo "${nc}${color1} $*${nc}"
}

err()
{
    echo "${nc}${color2} $*${nc}"
}

DESERTad=$(pwd)
DESERTfw=$DESERTad/../DESERT_Framework
M4FOLDER=$DESERTfw/DESERT/m4

for dir in $(ls -d */) ; do
    cd $DESERTad/$dir
    if [ -d m4 ] ; then
        cp $M4FOLDER/desert.m4 ./m4/
        info1 "desert.m4 updated in $dir"
    else
        err   "---There is no ./m4/ directory in $dir"
        err   "---Attempting subfolder exploration..."
	for ddir in $(ls -d */) ; do
            cd $DESERTad/$dir/$ddir
            if [ -d m4 ] ; then
                cp $M4FOLDER/desert.m4 ./m4/
                info1 "---desert.m4 updated in ${dir}${ddir}"
            else
                err   "---There is no m4 directory in ${dir}${ddir}"
            fi
	done
    fi
done



