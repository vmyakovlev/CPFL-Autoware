#!/bin/bash
#Please set below 2 arguments before execute this script.
# CUDA_VER   : 8.0, 9.0, etc....
# OS_VER     : 14.04, 16.04, etc...

REPOS_DIR=`echo "ubuntu${OS_VER}" | sed "s/\.//"`
wget -q https://developer.download.nvidia.com/compute/cuda/repos/${REPOS_DIR}/x86_64 -O index.html
CUDA_DEV=`cat index.html | grep cuda-repo-${REPOS_DIR}_${CUDA_VER} | grep amd64.deb | tail -n 1 | sed -e "s/</\n/g" -e "s/>/\n/g" | grep cuda-repo-${REPOS_DIR}_${CUDA_VER} | grep -v href`
wget -q https://developer.download.nvidia.com/compute/cuda/repos/${REPOS_DIR}/x86_64/${CUDA_DEV}
sudo dpkg -i ${CUDA_DEV}

function verInt(){ echo "$1" | sed "s/\.//" ; return 0 ; }
if [ `verInt ${CUDA_VER}` -ge `verInt 9.0` ] ; then
apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub ;
fi

sudo apt-get update
CUDA_SFX=`echo "${CUDA_VER}" | sed "s/\./\-/"`
apt-get install --yes -q cuda-${CUDA_SFX} 1>/dev/null
sudo /bin/sh -c 'echo "export PATH=/usr/local/cuda-${CUDA_VER}/bin:\${PATH}" >> ~/.bashrc'
sudo /bin/sh -c 'echo "export LD_LIBRARY_PATH=/usr/local/cuda-${CUDA_VER}/lib64:\${LD_LIBRARY_PATH}" >> ~/.bashrc'
source ~/.bashrc
