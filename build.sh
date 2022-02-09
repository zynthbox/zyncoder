#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

pushd $DIR
	if [ ! -d build ]; then
		mkdir build
	fi
	pushd build
		ZYNTHIAN_WIRING_LAYOUT="Z2_V2" cmake ..
		make
	popd
popd
