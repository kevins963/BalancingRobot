-----------
readme.txt
-----------
In this folder there are five applications tested on the
- NUCLEO-L053R8 RevC
- NUCLEO-L476RG RevC
- NUCLEO-F401RE RevC
- NUCLEO-F411RE RevC
boards:
  - SampleAppThT             Two STM32 Nucleo boards, one configured as Central (client),
                             the other as Peripheral (server), establish a connection.
                             After establishing the connection, by pressing the USER button
                             on one board, the LD2 LED on the other one gets toggled and
                             viceversa.
                             If the ThroughputTest configurations from the project options
                             menu are used, a test for the throughput estimation can be
                             performed.                             
  - SensorDemo               An STM32 Nucleo board acting as Peripheral (server) establishes 
                             a connection with a smartphone/tablet supporting BLE and with 
                             the "BlueNRG" app installed.
                             After establishing the connection, by pressing the USER button
                             on the board, the cube showed by the app on the smartphone/tablet
                             will rotate on the three axes (x,y,z).
	                         The application shows also how to add new GATT services and
                             characteristics.
  - Virtual_COM_Port         Application to be loaded on an STM32 Nucleo board in order to use
                             the tools for updating the BlueNRG firmware on the X-NUCLEO-IDB04A1
                             or X-NUCLEO-IDB05A1 expansion boards.
  - SampleApp_DMA_LowPower   Two STM32 Nucleo boards, one configured as Central (client),
                             the other as Peripheral (server), establish a connection.
                             After establishing the connection, by pressing the USER button
                             on one board, the micro on the other one gets toggled from RUN
                             to STOP mode and viceversa, while the STOP mode is set locally.
                             Current consumption could be monitored through an amperemeter
                             connected to JP6.
                             This sample application uses the STM32 Cube Low Level low power
                             optimizations along with the DMA module.
                             It allows the most efficient power consumption and is heavily
                             optimized.
                             It is less easy to understand compared to the other applications.
                             If you are interested in easier and more portable application using
                             low power, please download the OSXSmartConnPS package.
  - SensorDemo_DMA_LowPower  The low power version of the SensorDemo sample application described above.
                             This sample application uses the STM32 Cube Low Level low power
                             optimizations along with the DMA module.
                             It allows the most efficient power consumption and is heavily
                             optimized.
                             It is less easy to understand compared to the other applications.
                             If you are interested in easier and more portable application using
                             low power, please download the OSXSmartConnPS package.
To run these applications the BlueNRG expansion board (X-NUCLEO-IDB04A1 or X-NUCLEO-IDB05A1),
plugged on one of the supported STM32 Nucleo board, is needed.
Please, read the respective readme.txt file within the application folders for details 
and usage instructions.

IMPORTANT NOTE: To avoid issues with USB connection (mandatory if you have USB 3.0), it is suggested to
update the ST-Link/V2 firmware for STM32 Nucleo boards to the latest version. It can be done by downloading the
latest version of the STM32 ST-LINK utility and then use the ST-LINK -> Firmware update feature.

----------------  
Firmware Update
----------------
To improve the performance, the BlueNRG firmware of the X-NUCLEO-IDB04A1/X-NUCLEO-IDB05A1 can be updated to
their respective latest versions.
The tool and the readme with instructions for updating the BlueNRG firmware can be found in
folder Utilities/PC_Software/FlashUpdaterTool.
