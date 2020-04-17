start %~dp0/RoverController/bin/Debug/RoverController.exe
@echo off
For %%A in ("%USERPROFILE%") do (
    Set username=%%~nxA
)
@echo on
docker run -it --net=host -v /c/Users/%username%/.ssh:/root/.ssh jaybot_remote:latest
