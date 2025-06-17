## Ubuntu terminal vs GUI mode

### Terminal mode

Permanent
sudo systemctl set-default multi-user.target

Temporary
sudo systemctl isolate multi-user.target


### GUI mode

Temporary
sudo systemctl isolate graphical.target

Permanent
sudo systemctl set-default graphical.target
