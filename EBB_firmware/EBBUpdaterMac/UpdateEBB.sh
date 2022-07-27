#!/bin/bash

echo -e "Welcome to the EiBotBoard (EBB) Firmware Updater, version 2.8.1"
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
  printf "\nEntering bootloader mode"
  sleep 5
  #echo -e "Calling mphidflash to update firmware"
  ./mphidflash-1.8-osx-64 -w EBF_v281.hex -r
  printf "\n\nEnding update process; "
  sleep 2
  echo -e "Waiting for reset."

  i="10"

  while [ $i -gt 0 ]
    do

    result=`ls /dev | grep -m 1 usbmodem`
    portName=$base$result
    
    #echo -e "$portName"
    if [ "$result" ]
    then
      i="0"
    else
     # echo -e $i
     echo -e "."
     sleep 1
    fi

    i=$[$i-1]
    done

  printf "\n\nLooking for EBB."
  sleep 2
  result=`ls /dev | grep -m 1 usbmodem`
  portName=$base$result

  printf "\nAsking the EBB for its firmware version:\n"
  result2=`head -n 1 < $portName & echo -e "V" > $portName`

  if [[ $result2 == *"OK"* ]]
    then
      result2=`head -n 1 < $portName & echo -e "V" > $portName`
  fi

  printf "\n$result2"

  if [[ $result2 == *"2.8.1"* ]]
  then
    printf "\n\nFirmware updated successfully."
  else
    printf "\n\nFirmware update may not have succeeded."
  fi

else
  printf "\nChecking to see if we have a device in bootloader mode."
  ./mphidflash-1.8-osx-64 -w EBF_v281.hex -r
  sleep 5
  base='/dev/'
  result=`ls /dev | grep -m 1 usbmodem`
  portName=$base$result

if [ "$result" ]
 then
  printf "\nPreparing to check firmware version..."
  sleep 5
  printf "\nAsking the EBB for its firmware version:"
  result2=`head -n 1 < $portName & echo -e "V" > $portName`

  if [[ $result2 == *"OK"* ]]
    then
      result2=`head -n 1 < $portName & echo -e "V" > $portName`
  fi

  printf "\n$result2"

  if [[ $result2 == *"2.8.1"* ]]
  then
    printf "\n\nFirmware updated successfully."
  else
    printf "\n\nFirmware update may not have succeeded.\nYou might try running this again.\n"
  fi

else
  printf "\n\nFirmware update appears to have failed.\n"
fi

fi