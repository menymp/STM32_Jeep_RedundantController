# STM32_Jeep_RedundantController
Software for an rc self driving jeep with redundant validation

this controller is based on the stm32f103 microcontroller included in the well known "Blue pill" development board.

The idea is the communication through USB interface where the device will appear as Serial port on the raspberry pi SO.
With usb port the wire can be removed and change the main controller board (raspberry Pi)

the microcontroller give the raspberry pi acsess to the drivetrain motor speed and direction and the steering servo with a command serial interface.

the microcontroller program expect to receive commands for the drivetran often, if the wire brokes or the connection has been lost, the system will stop the motor as a security measure.


ToDo:
Electric diagrams
Include a CAN interface for a more flexible chassis expansion.
