set MsgGenExe=%~dp0\ros-sharp\Libraries\MessageGenerationConsoleTool\bin\Release\RosMsgGen.exe
%MsgGenExe% -s -r %~dp0\..\remote_ws\src\remote_app -o %~dp0\RoverController
%MsgGenExe% -s -r %~dp0\..\jaybot_ws\src\jaybot -o %~dp0\RoverController
