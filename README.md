This project aims to generate three phase separated pwm signals which can be filtered to obtain a sinusoidal waveform.

This is implemented on an IGBT inverter (H-Bridge) rated for 300V DC and 25A continuous current. Although it is rated for 300V, all tests were conducted at 15V DC.

The IGBT inverter accepts logic through a JST connector. Using this, each individual gate of the IGBTs of the H-bridge can be turned on and off. The inverter module has the gate driver circuitry. It drives the gate to 16V when a logic high is given and to -8V when logic low is given.

An ESP-32 was used to generate the PWM waveform at its GPIO pins. A trimmer was used to adjust the frequency of the output waveform.

The PWM was offloaded to the hardware with built in timer and comparator, called the MCPWM peripheral. The goal was to achieve uniform CPU usage, by performing any required calculations in the time interval between the PWM updates.

The main structure had the following tasks, functions, queues, ISR:
1. Duty_filler task
2. input_check task
3. pwm_ISR ISR
4. mcpwm_init_pwm function
5. duty_queue queue
6. generate_SinLUT function

Working of the code:

The CPU first runs the generate_SinLUT function which generates a sinusoidal lookup table of a pre-defined size. Then the tasks duty_filler, input_check and the queue duty_queue are declared. input_check is a temporal task whereas duty_filler is an event driven task.

The input_check code polls the gpio pin through its ADC to get the value of the potentiometer. With this value it decides the phase_step variable. This variable is responsible for affecting the frequency, and it decides how fast should the CPU read the LUT.

Whenever the mcpwm timer hits hits its end and restarts (end of one pwmp_period) an interrupt is called which is handled by pwm_ISR. pwm_ISR pops values from the duty_queue queue and updates the value of the comparator. It is important to do it this way since FPU calculations cannot be done in ISR. It also checks the length of the duty_queue, if it is less than 1/3rd of its value, duty_filler task is called.

duty_filler task, based on the current phase and the phase_step variable, traverses the LUT and converts its values into meaningful comparator values, stored in a custom struct and sent to a queue. This task keeps on running until it fills up the queue. It also yields for other tasks using vTaskDelay to avoid Task Watchdog timeout error.

