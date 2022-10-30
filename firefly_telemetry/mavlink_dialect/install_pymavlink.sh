cd "$(dirname "$0")"

if [ -f "firefly_dialect.md5" ] && md5sum --status -c firefly_dialect.md5
then
    exit
fi

if [ "$ROS_DISTRO" = "noetic" ]
then
(cd ../../pymavlink && exec sudo MDEF=$(pwd)/message_definitions python3 -m pip install . -v --upgrade)
else
(cd ../../pymavlink && exec sudo MDEF=$(pwd)/message_definitions python2 -m pip install . -v --upgrade)
fi

if [ "$?" = 0 ]
then
md5sum firefly.xml > firefly_dialect.md5
fi
