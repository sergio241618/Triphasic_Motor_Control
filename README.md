ESP32 SPWM Motor Controller

This firmware implements a 3-phase Sinusoidal PWM (SPWM) generator for controlling an AC motor. It uses an event-driven, multi-task architecture based on FreeRTOS task notifications.

Control commands (Frequency, Amplitude, RPM Reference) are received from a host PC via UART.

1. UART Communication Settings

The firmware's UART task is configured with the following settings:

-Baud Rate: 115200
-Data Bits: 8
-Parity: None
-Stop Bits: 1
-Flow Control: None

2. UART Command Protocol

The firmware accepts ASCII string commands, terminated by a newline character (\n).

Command Format:
The general format follows the SET [CHANNEL] [VALUE_LIST] structure.

SET [CHANNEL] [VALUE_LIST]

-SET: The literal string "SET".

-[CHANNEL]: The target control parameter. Note: This does not map to a single motor phase.
    CH1: Controls the Global Frequency of the SPWM output (in Hz).
    CH2: Controls the Global Amplitude of the SPWM output (as a percentage, 0-100).
    CH3: Sets the Reference RPM value (used for telemetry/control loops).

-[VALUE_LIST]: A single uint32_t integer value OR a comma-separated list of uint32_t integer values.

Examples:
Single Value Commands:

# Set the global frequency to 60 Hz
SET CH1 60

# Set the global amplitude to 100%
SET CH2 100

# Set the reference RPM to 3600
SET CH3 3600

Multiple Value (Sequence) Command:

# Set frequency to 10Hz, then 20Hz, then 30Hz, then 0Hz
SET CH1 10,20,30,0

3. Implementation of Multiple Value Sequences

The system handles command sequences by splitting the work between the UART task and the target PWM/Control task.

a) UART Task (uart_rx_task)

Parse: When a command (e.g., SET CH1 10,20,30) is received, the uart_rx_task parses the [VALUE_LIST] string ("10,20,30").

Split: It splits the string by the comma (,) delimiter.

Pack: It packs all the parsed uint32_t values into a single seq_t struct (which contains an array vals[] and a len).

Enqueue: This single seq_t struct is sent to the corresponding command queue (cmdQueueA, cmdQueueB, or cmdQueueC) using xQueueOverwrite(). This ensures the task only processes the most recent sequence.

b) Control Task (pwm_task)

Check Queue (Non-Blocking): The pwm_task is normally blocked, waiting for a notification from the SPWM timer. At the beginning of its loop, it performs a non-blocking check (xQueueReceive(..., 0)) on its command queue.

Execute Sequence: If it finds a new seq_t struct, it enters a blocking for loop to execute the sequence.

Hold for T_pulse: Inside this loop, it applies the first value (e.g., phases::set_frequency(10)).

It then blocks for a fixed interval (vTaskDelay(pdMS_TO_TICKS(T_PULSE_MS)), where T_PULSE_MS is 250ms).

It then applies the next value from the struct's array and delays again.

This process repeats until all values in the sequence have been applied.

Resume SPWM: Once the sequence is finished, the task resumes its normal operation, returning to step 1 and blocking on the timer notification (ulTaskNotifyTake) to continue generating the SPWM signal.