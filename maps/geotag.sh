#!/bin/bash



file=$1

[[ -n "$1" ]] || file=*.tiff

for tifffile in $file
do
    basefile=${tifffile%.*};
    geotagfile=${basefile}'.geotag';
    if [ -f $geotagfile ]
    then
        echo "Geotagging file $tifffile with tag $geotagfile"
        cp $tifffile /tmp/
        geotifcp -g $geotagfile /tmp/$tifffile $tifffile
        rm /tmp/$tifffile
    else
        echo "ERROR: No geotag information for file $tifffile !! Expecting to find $geotagfile"
    fi
done
