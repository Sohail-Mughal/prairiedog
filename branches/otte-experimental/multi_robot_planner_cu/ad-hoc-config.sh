# remember to turn off networking first
echo "Enter the desired ip of this computer: "
read theip
ifconfig wlan1 down
ifconfig wlan1 up
iwconfig wlan1 essid robots mode ad-hock
ifconfig wlan1 $theip
