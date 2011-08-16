# remember to turn off networking first
echo "Enter the desired ip of this computer: "
read theip
ifconfig wlan0 down
ifconfig wlan0 up
iwconfig wlan0 essid robots mode ad-hock
ifconfig wlan0 $theip
