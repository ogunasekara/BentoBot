import serial
import threading
import time
import csv

FILE_PREFIX = "left_wheel_new"

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
data = [] # list of dicts, fields are motor_cmd, ang_vel

#--- READING AND PARSING ---#

vel_sum = 0
vel_cnt = 0

# thread to read serial continuously for parsing purposes
def read_serial():
    global vel_sum
    global vel_cnt

    while True:
        try:
            line = ser.readline().decode()
            measurement = parse_line(line)

            vel_sum += measurement # add target measurements
            vel_cnt += 1

            #print(measurements)
        except UnicodeDecodeError:
            pass

# daemon thread will close on end of main thread
read_thread = threading.Thread(target=read_serial, daemon=True)
read_thread.start()

def parse_line(line):
    line = line.strip()
    return float(line)

def reset_avg_vel():
    global vel_sum
    global vel_cnt
    
    vel_sum = 0
    vel_cnt = 0

def get_avg_vel():
    global vel_sum
    global vel_cnt

    return (vel_sum * 1.0) / (vel_cnt * 1.0) 

#--- COMMANDS ---#

def set_motor_pwm(pwm):
    s = str(pwm) + '\n'
    ser.write(s.encode())
    time.sleep(0.05)

def save_to_csv(filename, data):
    filename = FILE_PREFIX + "_" + filename
    fieldnames = {'motor_cmd', 'ang_vel'}

    with open(filename, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for row in data:
            writer.writerow(row)

#--- MAIN LOOP ---#

try:
    set_motor_pwm(0)

    print("waiting 5 seconds before negative")
    time.sleep(5)

    print("starting positive loop")
    # start from top and go down
    for cmd in range(-255, 0, 5):
        print("cmd: ", cmd)
        reset_avg_vel()
        set_motor_pwm(cmd)
        time.sleep(2)
        avg_vel = get_avg_vel()
        data.append({'motor_cmd': cmd, 'ang_vel': avg_vel})
        print("avg_vel: ", avg_vel)

    set_motor_pwm(0)
    print("waiting 5 seconds before positive")
    time.sleep(5)

    for cmd in range(255, 0, -5):
        print("cmd: ", cmd)
        reset_avg_vel()
        set_motor_pwm(cmd)
        time.sleep(2)
        avg_vel = get_avg_vel()
        data.append({'motor_cmd': cmd, 'ang_vel': avg_vel})
        print("avg_vel: ", avg_vel)

    set_motor_pwm(0)

    save_to_csv('test.csv', data)

except KeyboardInterrupt:
    ser.close()
