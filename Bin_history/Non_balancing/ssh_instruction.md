To connect to the Raspberry PI remotely using SSH we follow the following procedure

### 1. On pi (if not done already:

```sudo raspi-config``` --> And from there enable SSH in Interface

Use: ```hostname -I``` To find the IP address


### 2. On laptop's terminal

```ping raspberrypi.local -4``` To find the IP address from the laptop (-4 allows to get the traditional format for IP)

```ssh pi@<pi-ip-address>``` eg: ```ssh pi@192.168.1.42```

Alternatively on mac you can type: ```alias pi="ssh pi@192.168.1.42"``` and then just type ```pi``` (The previous method also works)

You might be asked for the password of your Raspberry Pi. 
