@echo off
start cmd /k "mavproxy --master="com42",57600 --out 127.0.0.1:14560 --out 127.0.0.1:14561 --out 127.0.0.1:14562 --out 127.0.0.1:14563"
start cmd /k "python C:\Users\YashikaAchari\pymavlink\GCSmain.py"

REM start mavproxy.exe
REM note: change com port as required 