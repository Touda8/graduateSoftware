@echo off
cd /d F:\project\graduateSoftware
echo [BUILD] Starting UIC/MOC...
D:\software\anaconda\Library\bin\uic.exe twoProjector.ui -o src\ui_twoProjector.h
D:\software\anaconda\Library\bin\moc.exe src\MainWindow.h -o src\moc_MainWindow.cpp
D:\software\anaconda\Library\bin\moc.exe src\visualization\VtkWidget.h -o src\visualization\moc_VtkWidget.cpp
D:\software\anaconda\Library\bin\moc.exe src\common\ProjectorManager.h -o src\moc_ProjectorManager.cpp
echo [BUILD] Running MSBuild...
"C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\MSBuild\Current\Bin\MSBuild.exe" twoProjector.vcxproj /p:Configuration=Release /p:Platform=x64 /m /v:minimal
echo [BUILD] Exit code: %errorlevel%
