#angle of each explicit wheel wrt straight forward reference
float64[] angle 

#speed of each drive wheel
float64[] speed

#pwm (12 bit 0 to 4095) to each motor (8 motors of drive system)
int16[] pwm 

#direction value, +1 if speed > 0, -1 if speed < 0 and =0 if speed = 0
int8[] direction

#sys_check boolean
bool sys_check