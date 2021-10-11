#!/bin/bash
path=`pwd`
for dir in $path/../resource/models/*; do
    cd $dir
    echo $dir
    xmacro4sdf model.sdf.xmacro
done