# STM32-DC-Motor-Control
This project presents a DC motor control system based on the STM32 microcontroller, utilizing a predictive open-loop control method. By adjusting the duty cycle and duration of the PWM signal, the system estimates motor speed without relying on a closed-loop feedback mechanism. Speed measurements are collected using an encoder to build a mathematical model for fine-tuning control parameters. This project is especially useful for embedded systems applications where simplicity and low resource consumption are critical.
Designed for academic and automotive applications.
Technical specifications:
I used Timer2 for the encoder mode (for signals coming from A & B)
Timer3 to generate the PWM signal (using  "__HAL_TIM_SET_COMPARE " we can put the value in the CCR (counter compare register) ,considering the count 65535 is equal to 100% (duty cycle))
