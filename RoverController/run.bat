@echo off

for %%A in ("%USERPROFILE%") do (
    set USERNAME=%%~nxA
)

set TEMP_LOC=%USERPROFILE%\AppData\Local\Temp\jaybot.tmp

docker run -d --net=host -v /c/Users/%USERNAME%/.ssh:/root/.ssh jaybot_remote:latest > %TEMP_LOC%
set /P DOCKER_ID=<%TEMP_LOC%

docker exec %DOCKER_ID% /bin/bash -c "ip addr show eth2 | grep -Po 'inet \K[\d.]+'" > %TEMP_LOC%
set /P WS_ADDR=<%TEMP_LOC%

set ROS_MASTER_URI=http://%WS_ADDR%:11311
python -c "import socket; hostname=socket.gethostname(); print(socket.gethostbyname_ex(hostname)[-1][-1])" > %TEMP_LOC%
set /P ROS_IP=<%TEMP_LOC%
del %TEMP_LOC%

echo Waiting for ROS to initialize...
timeout /t 7

%~dp0\RoverController\bin\Release\RoverController.exe

echo Waiting for ROS to close gracefully...

docker stop %DOCKER_ID%

@echo on
