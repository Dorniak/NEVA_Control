# Setup mjpg-streamer on Ubuntu Server 20

## List your webcams
sudo ls -ltrh /dev/video*

## Install from snap and grant permission
sudo snap install mjpg-streamer
sudo snap connect mjpg-streamer:camera

## Start server
#### mjpg-streamer -i "input_uvc.so -d /dev/video0 -n -r 800x600 -f 15 -l auto" -o "output_http.so -p 9090 -w /home/epochs/Desktop/www"

## Url
#### http://localhost:9090/?action=stream