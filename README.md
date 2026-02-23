# ESP32 CAN-Sniffer

## Prebuild NVS for WiFi connection

To setup WiFi-Connection in advance (SSID/PSK) you could put them into a file, build an bin-image of the NVS partition and flash it to the chip. This only need to be done once.

Follow these steps:

1.) edit the file `nvs.csv` and replace the placeholder values with your WiFi credentials

2.) Connect your ESP32 to the PC and open an `ESP-IDF Terminal`. Ensure the environment variable `$IDF_PATH`, pointing to your ESP-IDF installation is set properly:
```
echo "$env:IDF_PATH"
```

Find the START address and SIZE of the NVS partition. We are using the "Two OTA" from the ESP-IDF.

Use this command to get that START address:
```
idf.py partition-table
```
from the output get the 4th param in the line "nvs" as START (0x9000):
```
nvs,data,nvs,0x9000,24K,
```

Get the SIZE of partition in HEX by issuing this command:
```
cat "$env:IDF_PATH\components\partition_table\partitions_two_ota_large.csv"
```
The SIZE is the last param in the line of "nvs" (0x6000):
```
nvs,      data, nvs,     ,        0x6000,
```

3.) build a binary image of the data (using the SIZE in HEX)
```
"$env:IDF_PATH\components\nvs_flash\nvs_partition_generator nvs_partition_gen.py" generate nvs.csv nvs.bin 0x6000
```

3.) Flash the NVS partition to the ESP`
(set COM-Port to the one registered with your ESP)
```
esptool.py --port COM5 write_flash 0x9000 nvs.bin
```
