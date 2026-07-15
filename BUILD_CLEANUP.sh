#!/bin/bash

set -e

[ -d ./local_usr ] && rm -fR ./local_usr
[ -d ./libmodbus ] &&
{
  pushd ./libmodbus
  [ -f Makefile ] &&
  {
    make uninstall
    make clean
  }
  popd
  rm -fR ./libmodbus
}

