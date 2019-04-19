#!/bin/bash

echo -e "Welcome to the EiBotBoard (EBB) Firmware Updater, version 2.6.2"
echo -e "\nIn case of unexpected results, please make sure that you have"
echo -e "exactly one EBB connected to your computer via USB."

base='/dev/'
result=`ls /dev | grep -m 1 usbmodem`
portName=$base$result

#echo -e "$portName"
if [ "$result" ]
then
  printf "\n\nEBB appears to be located; Beginning update process. "

  echo -e "BL" > $portName
  sleep 2s
  #echo -e "Calling mphidflash to update firmware"
  ./mphidflash -w EBF_v262_BL.unified.hex -r
  printf "\n\nFirmware update written; preparing to test."
  sleep 5s

  printf "\n\nLooking for EBB."
  
  result=`ls /dev | grep -m 1 usbmodem`
  portName=$base$result

  printf "\nAsking the EBB for its firmware version:"
  echo -e "V" > $portName
  result2=`head -n 1 < $portName`
  printf "\n$result2"

  if [[ $result2 == *"2.6.2"* ]]
  then
    printf "\n\nFirmware updated successfully."
  else
    printf "\n\nFirmware update may not have succeeded."
  fi

else
  printf "\nChecking to see if we have a device in bootloader mode."

  ./mphidflash -w EBF_v262_BL.unified.hex -r
  sleep 2s
  base='/dev/'
  result=`ls /dev | grep -m 1 usbmodem`
  portName=$base$result

if [ "$result" ]
 then
  printf "\nFirmware burn complete. Preparing to test."
 
  sleep 5s
  printf "\nAsking the EBB for its firmware version:"
  echo -e "V" > $portName
  result2=`head -n 1 < $portName`
  printf "\n$result2"

  if [[ $result2 == *"2.6.2"* ]]
  then
    printf "\n\nFirmware updated successfully."
  else
    printf "\n\nFirmware update may not have succeeded."
  fi

else
  printf "\n\nFirmware update appears to have failed."
fi

fi