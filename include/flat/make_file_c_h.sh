#!/bin/bash
# https://github.com/dvidelabs/flatcc/blob/master/doc/flatcc-help.md
rm *.h

flatc --cpp Heartbeat.fbs &
flatc --cpp RequestConnection.fbs &
flatc --cpp ResponseConnection.fbs &
