#*******************************************************************
#                   MODBUS TCP PID CONTROL - POLLING
#*******************************************************************
'''
File name: PID_Control_v2.py
Description: This program implements a Modbus TCP client with
    PID control. This model is recommended to stable set point.
URL to Download:
    https://github.com/LeandroTeodoroRJ/FX3U-PLC/tree/main/FXCPU_Examples
Stable: Yes
Version: 2.0.0
Current: Yes
Maintainer: leandroteodoro.engenharia@hotmail.com
Depends:
    pyModbusTCP==0.3.0
Architecture: X86
Compile/Interpreter: python 3.1.12
Access: Public
Changelog: No changelog
Readme: No readme (inside this code)
Document Extra-Files:
    MODBUS_TABLE_CONTROL.txt    Summary modbus table addresses
Links:
    pyModbus Documentation - https://pypi.org/project/pyModbusTCP/
Other Notes:
    * Run this as root to listen on TCP privileged ports (<= 1024).
    * To run
        sudo <full interpreter path> ./PID_Control.py
        sudo /home/leandro/Dev/py3-10_modbusTCP/bin/python3 ./PID_Control_v2.py

'''

import time

from pyModbusTCP.client import ModbusClient

SERVER_HOST = "192.168.0.101"
SERVER_PORT = 502

# TCP auto connect on modbus request
tcp_comm = ModbusClient(host=SERVER_HOST, auto_open=True, unit_id=1, port=SERVER_PORT)

#PID Control config
kp=5        #Porportional coefficient
ki=0.4     #Integral coefficient
kd=0      #derivative coefficient
error_coefficient=5     #Error controller trhesrold
limit_output=1000       #Maximum controller limit_output
limit_lower=0           #Minimum controller output
last_messure=0          #last feedback value
last_output=0           #Last control output
time_sample=0.2           #Loop time aquisition
sp=0
feedback=0
last_output=0
last_messure=0
last_error=0
old_mesure=0

# Modbus Table
START_READ_ADDRESS=0
NUMBER_REG_TO_READ=3
PID_OUTPUT_CONTROL=3
LEVEL_METER=0
SET_POINT=2

def pid_control(setpoint, error, max_output, min_output, measure):
    global last_output
    global last_messure
    global last_error
    global old_mesure
    error_meas = setpoint - measure     #real time error
    if (abs(error_meas) < error):
        return last_output              #Return without control calc

    c1 = kp*(error_meas-last_error)
    c2 = ki*time_sample*error_meas
    c3 = (kd/time_sample)*(measure-2*last_messure+old_mesure)
    PID = last_output + c1 + c2 + c3
    output=PID

    #Limits outputs test
    if (output > max_output):
        output = max_output
    if (output < min_output):
        output = min_output

    last_error=error_meas
    old_mesure=last_messure
    last_messure=measure
    last_output=output
    return output

#Start service
while True:
    #Read holding registers
    regs = tcp_comm.read_holding_registers(START_READ_ADDRESS, NUMBER_REG_TO_READ)
    if regs:
        feedback=regs[LEVEL_METER]
        sp=regs[SET_POINT]

        #PID control fuction
        pid_value=pid_control(sp, error_coefficient, limit_output, limit_lower, feedback)
        actuator_value=int(pid_value)

        #Write actuator register
        tcp_comm.write_single_register(PID_OUTPUT_CONTROL, actuator_value)

    else:
        exit()

    # wait time sample
    time.sleep(time_sample)
