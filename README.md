Design Objectives
  Accurately capture voltage and current signals
  Establish an efficient, safe, and low-interference data transmission mechanism
  Provide stable and usable time-series data for backend system processing

Core Technologies
Signal Acquisition:
  Utilize the MCU’s built-in ADC to digitize voltage and current signals
  System is divided into a voltage acquisition module and a current acquisition module

Voltage Acquisition Module:
  Implements a voltage divider circuit to scale down high-voltage signals
  Ensures signal levels fall within the safe operating range of the MCU ADC, improving measurement safety and stability

Current Acquisition Module:
  Uses a Current Transformer (CT) for current sensing
  Combines full-wave rectification and filtering to convert AC signals into stable DC voltage suitable for ADC input

Data Transmission Interface:
  Adopts CAN bus as the communication protocol, offering high noise immunity and real-time performance
  Enables reliable data transmission between the MCU and the host PC

![image](https://github.com/user-attachments/assets/7b730eb7-6d5e-41fb-aa32-3f42efc5dd7c)
![前端取樣](https://github.com/user-attachments/assets/1c8c9187-8ac6-42cb-9463-3bc4454b9514)


