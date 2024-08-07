# VKSDK-MODBUS
This codebase was designed to incorporate the vkl-m12 system along with a radxa zero 3w. UART4 is assumed to be used as well GPIO pin 113 (gpiochip 3 pin 17).

## Setup
1. Install vk-system.

2. Edit vkl-m12-color_arm.json file.
     Edit path to blob file
     ```
     "blob_path": "<path_to_yolov8s_openvino_2022.1_7shave.blob>",
     ```

3. Install dependencies 
    ```
    sudo apt install libgpiod-dev
    ```
4. Set up UART4 on Radxa zero 3w
    ```
    sudo armbian-add-overlay rk356x-uart4-m1.dts
    sudo reboot now
    ```

## Compiling 
```
mkdir build && cd build
cmake ..
make
```

## Running the program 
1. Start camera driver using the config file "vkl-m12-color_arm.json"
2. Run ```sudo ./main <modbus_id> ``` e.g. ```sudo ./main 10 ```

## Reading from modbus 
MODBUS RTU PROTOCOL
|  Field  | Value |
| ------ |:-----:|
|  Baudrate  |  115200      |
| Databits     | 8    |
| Parity      | even     |
| Input register data type | 16-bits|
| Modbus ID | _user defined_ |
| Input register | 1 - Detection, "1" for target detected "0" for no target detected <br> 2 - Heartbeat of camera driver, <br>"1" for camera driver heart beat received, <br>"0" for no heartbeat received  |

**When there is no heartbeat received from camera driver, Detection value should be thrown away.


## Operating System used 

Linux radxa-zero3 6.1.43-vendor-rk35xx #1 SMP Thu Jun 20 10:33:36 UTC 2024 aarch64 GNU/Linux

Distributor ID:	Debian
Description:	Armbian 24.5.3 bookworm
Release:	12
Codename:	bookworm
