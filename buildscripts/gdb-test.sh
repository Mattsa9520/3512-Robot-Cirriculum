#!/bin/bash
set -e

./buildscripts/remove-logs.sh

./gradlew installFrcUserProgramTestLinuxx86-64DebugGoogleTestExe $@
sed -i 's/exec /gdb /' build/install/frcUserProgramTest/linuxx86-64/debug/frcUserProgramTest
cd build/install/frcUserProgramTest/linuxx86-64/debug && ./frcUserProgramTest