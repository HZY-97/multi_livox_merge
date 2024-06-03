#!/bin/bash

killProcess(){
    echo "收到 Ctrl+C 信号，正在结束进程..."
    kill -9 $pid_multi_livox_merge
    
    if [ "$Rviz2_process" == "true" ]; then
        kill -9 $pid_rviz2
    fi
    exit 0
}

source ./setenv.bash

echo $MULTI_LIVOX_MERGE_INSTALL_PATH

Rviz2_process=false

readonly USE_RVIZ2="rviz"

if [ $# -eq 1 ] && [ "$1" == "rviz" ]; then
    Rviz2_process=true
else
    Rviz2_process=false
fi

# every process wait time 0.2s
wait_time=0.2s

# config check
if [ -f "$MULTI_LIVOX_MERGE_INSTALL_PATH/../config/livox_config.json" ]; then
    echo "config/livox_config.json 文件已存在."
else
    echo "config/livox_config.json 文件不存在."
    # 复制当前目录下的backup_config/config目录到上一层目录并重命名为config
    cp -r $MULTI_LIVOX_MERGE_INSTALL_PATH/backup_config/config $MULTI_LIVOX_MERGE_INSTALL_PATH/..
    echo "已将 backup_config/config 目录复制到 $MULTI_LIVOX_MERGE_INSTALL_PATH/../config"
fi

multi_livox_merge=$MULTI_LIVOX_MERGE_INSTALL_PATH/bin/multi_livox_merge
$multi_livox_merge &
pid_multi_livox_merge=$!
echo "multi_livox_merge PID:$pid_multi_livox_merge"
sleep $wait_time

if [ "$Rviz2_process" == "true" ]; then

    rviz_config=$MULTI_LIVOX_MERGE_INSTALL_PATH/rviz/multi_lidar.rviz
    # rviz2 -d $rviz_config > /dev/null 2>&1 & # > /dev/null 将标准输出重定向到空设备  2>&1 将标准错误也重定向到标准输出
    rviz2 -d $rviz_config > /dev/null &
    pid_rviz2=$!
    echo "rviz2 PID:$pid_rviz2"
fi

sleep 3
echo "*************************************"
echo "**********All Process Start**********"
echo "*---PID------NAME-------------------*"
echo "* $pid_multi_livox_merge     multi_livox_merge             *"
if [ "$Rviz2_process" == "true" ]; then
    echo "* $pid_rviz2     rviz2                   *"
fi
echo "*************************************"
echo "you can use Ctrl+C to shutdown all process"
echo "if some process can't exit successfuly, you can press"
if [ "$Rviz2_process" == "true" ]; then
    echo "kill -9 \
$pid_multi_livox_merge \
$pid_rviz2"
else
    echo "kill -9 \
$pid_multi_livox_merge"
fi

trap killProcess SIGINT SIGTERM

while true; do
    sleep 1
done
