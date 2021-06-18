#!/bin/bash

echo "Start setting ..."

basepath=$(cd `dirname $0`; pwd)
sourcepath=$basepath"/defaultconfig"
savepath=$basepath"/cache"

if [ ! -d $savepath ]; then
  mkdir -p ${savepath}
else
  echo "Folder exist"
fi

cd $basepath"/cache"
mkdir wav
mkdir pcm
mkdir log
echo "Setting finish!"
