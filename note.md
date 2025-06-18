## Ubuntu terminal vs GUI mode

### Terminal mode

Permanent
```bash
sudo systemctl set-default multi-user.target
```

Temporary
```bash
sudo systemctl isolate multi-user.target
```


### GUI mode

Temporary
```bash
sudo systemctl isolate graphical.target
```

Permanent
```bash
sudo systemctl set-default graphical.target
```
