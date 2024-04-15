#!/bin/bash

if [ -z "$PS1" ]; then
  echo "交互模式"
  SCRIPT_DIR="$( cd "$( dirname "$0" )" && pwd )"
else
  echo "非交互模式"
  SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
fi

export MULTI_LIVOX_MERGE_INSTALL_PATH=$SCRIPT_DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MULTI_LIVOX_MERGE_INSTALL_PATH/lib

echo "当前multi_livox_merge安装路径为: $MULTI_LIVOX_MERGE_INSTALL_PATH"