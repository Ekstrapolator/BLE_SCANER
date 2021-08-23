
# ESP32 debuging steps

## prerequisites

copy and update lunch.json file 
copy debug file to project you wish to debug

## debuging command

openocd -f debug\ftdi_ft2322.cfg -f debug\esp32-wrover-kit-3.3v.cfg
