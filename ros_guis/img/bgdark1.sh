#!/bin/bash

gsettings set org.gnome.desktop.background picture-options 'centered'
gsettings set org.gnome.desktop.background primary-color "#242943"
f1="$(pwd)/sze_242943.png"
echo "Setting new background"
echo $f1
gsettings set org.gnome.desktop.background picture-uri $f1
