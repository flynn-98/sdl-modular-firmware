# SDL Board Firmwares
Firmware repository for self-driving lab modules

## Using Platformio to Flash Latest Firmware

Install the [PlatformIO VSCode Extension](https://docs.platformio.org/en/latest/integration/ide/vscode.html) and open a new Pio terminal (found in *Quick Access/Miscellaneous*). Change directory to either *gantry-kit* or *fluid-handling-kit* in the terminal, then connect the the target Arduino Nano via USB and run the following command:

```
cd ec-ms-controller/
```

```
pio run -t upload -e serial