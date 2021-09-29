#!/bin/bash

gsettings set org.gnome.desktop.background picture-options 'centered'
gsettings set org.gnome.desktop.background primary-color "#FFFFFF"
f1="$(pwd)/sze_jkk_sztaki_zala_ffffff.png"
echo "Setting new background"
echo $f1
gsettings set org.gnome.desktop.background picture-uri $f1
