@echo off
rmdir /Q /S %USERPROFILE%\.ssh
mkdir %USERPROFILE%\.ssh
ssh-keygen -t rsa -f %USERPROFILE%\.ssh\id_rsa -P ""
echo Please Wait!
%~dp0\build_docker.bat
@echo on
