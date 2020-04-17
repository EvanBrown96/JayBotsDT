@echo off

for %%A in ("%USERPROFILE%") do (
    set USERNAME=%%~nxA
)

set TEMP_LOC=%USERPROFILE%\AppData\Local\Temp\jaybot.tmp

docker run -d --net=host -v /c/Users/%USERNAME%/.ssh:/root/.ssh jaybot_remote:latest > %TEMP_LOC%
set /P DOCKER_ID=<%TEMP_LOC%

docker exec %DOCKER_ID% /bin/bash -c "ip addr show eth2 | grep -Po 'inet \K[\d.]+'" > %TEMP_LOC%
set /P WS_ADDR=<%TEMP_LOC%

rm %TEMP_LOC%

%~dp0/RoverController/bin/Debug/RoverController.exe

docker stop %DOCKER_ID%

@echo on
