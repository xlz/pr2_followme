#!/bin/sh
exec schroot -c hydro -d /home/xlz/followme -- sh -c ". devel/setup.sh; sleep 10; rosrun followme tracker $@"
