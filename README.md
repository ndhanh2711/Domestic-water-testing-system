# Water Quality Monitoring System

## Introduction
The water quality monitoring system is designed to measure and monitor key water parameters such as Total Dissolved Solids (TDS) and pH. If the parameters do not meet the required standards, the system will activate a pump to flush out the water that doesn't meet the quality criteria. The system also tracks the flow rate of water being pumped into the storage tank, calculates the flow rate and water volume, and displays these parameters on an LCD screen. Additionally, the system supports data monitoring and analysis via the Blynk IoT application.

## System Components
1. **TDS Sensor**: Measures the Total Dissolved Solids (TDS) in the water. If the TDS value exceeds 1000ppm and the pH is outside the range of 6 to 8.5, the system will activate the pump to flush out the water that does not meet the standard.
2. **pH Sensor**: Measures the pH level of the water, which helps determine if the water is acidic or alkaline. Water is considered unsuitable if the pH is below 6 or above 8.5.
3. **Flow Meter**: Measures the flow rate of water being pumped into the storage tank, helping to calculate the flow rate and water volume.
4. **LCD Screen**: Displays the measured parameters from the sensors, including TDS, pH, flow rate, and water volume.
5. **Blynk IoT Application**: Monitors the system parameters through the Blynk app on a smartphone. The app also allows for graphing the water flow data for the past 30 days and calculating the total water volume pumped in a month.

## How It Works
1. **Measuring TDS and pH**: The TDS and pH sensors continuously send data to the microcontroller. If the TDS exceeds 1000ppm or the pH is outside the range of 6 to 8.5, the pump is activated to flush out the water that does not meet the standard.
   
2. **Measuring Water Flow**: The flow meter measures the water flow rate being pumped into the storage tank. This data is used to calculate the water volume in the tank.

3. **Displaying Parameters on LCD**: All parameters, including TDS, pH, flow rate, and water volume, are displayed on the LCD screen for real-time monitoring.

4. **Monitoring via Blynk IoT**: The system sends data to the Blynk IoT app, where users can monitor key parameters, view flow rate graphs for the past 30 days, and calculate the total water pumped during the month.

## Key Features
- **Water Quality Monitoring**: Measures TDS and pH levels to ensure water quality.
- **Automatic Fault Handling**: When the water does not meet the required standard, the pump automatically flushes out the water.
- **Water Flow Monitoring**: The flow meter measures and calculates the amount of water being pumped into the storage tank.
- **LCD Display**: Displays the measured parameters clearly on the LCD screen.
- **Monitoring and Analysis via Blynk IoT**: The Blynk IoT app allows for monitoring and analyzing data, including generating water flow graphs for the past 30 days and calculating the total water volume for the month.

## Hardware Requirements
- TDS Sensor
- pH Sensor
- Water Flow Sensor
- LCD Screen
- ESP32 or compatible microcontroller
- Flush Pump
- Blynk IoT Application (requires a Blynk account)

## Installation
1. Install the Blynk IoT software and create a new project.
2. Configure the sensors with the ESP32 microcontroller.
3. Program the microcontroller to process signals from the sensors and send data to the Blynk app.
4. Connect the LCD screen to display the measured parameters.
5. Connect the flush pump to the microcontroller to automatically flush out water when it does not meet the required standard.

## Blynk Application
Download and install the Blynk app from [Google Play](https://play.google.com/store/apps/details?id=com.blynk.android) or [App Store](https://apps.apple.com/us/app/blynk/iD1043668946).
- Create a new project and obtain your authentication token (Token).
- Configure the interface with widgets like graphs, indicators, and pump control.

## Contact
If you have any questions or ideas, please contact via email: [nguyenhanh2711abc@gmail.com].

### Reference Documentation
TDS Sensor Code: https://randomnerdtutorials.com/esp32-tds-water-quality-sensor/

Ph0-14 Document:
1. https://how2electronics.com/diy-iot-water-ph-meter-using-ph-sensor-esp32/
2. https://www.amazon.com/GAOHOU-PH0-14-Detect-Electrode-Arduino/dp/B0799BXMVJ
