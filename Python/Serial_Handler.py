import os
import serial
import time
from collections import Counter

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) 
time.sleep(2)  

# Create a named pipe (FIFO)
fifo_path = 'mypipe'
if not os.path.exists(fifo_path):
    os.mkfifo(fifo_path)

direction_counter = Counter()

def send_data(data):
    try:
        ser.write(data.encode())
        print(f'Sent: {data}')
        time.sleep(0.5)
        if ser.in_waiting > 0:
            ack = ser.readline().decode('utf-8').rstrip()
            if ack == "ACK":
                print(f'Received acknowledgment for: {data}')
                return True
            else:
                print(f'Unexpected response: {ack}')
    except Exception as e:
        print(f'Error sending data: {e}')
    return False 

def get_sensor_data():
    if send_data("D"):
        time.sleep(0.1)
        while ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').rstrip()
            print(f'Received sensor data: {data}')

def write_most_common_direction(voltage=None, ldr=None):
    if direction_counter:
        most_common_direction, count = direction_counter.most_common(1)[0]
        with open("msm.txt", "w") as file:
            file.write(f"Final Direction: - {most_common_direction}\n")
            if voltage is not None:
                file.write(f"Voltage: - {voltage}\n")
            if ldr is not None:
                file.write(f"LDR: - {ldr}\n")
            file.write(f"Count: - {count}\n")
        print(f"Updated most common direction in msm.txt: {most_common_direction} (Count: {count})")

def main():
    print("Start")
    ser.write("I".encode())
    # Monitor FIFO for directions
    while True:
        # Read from the FIFO
        with open(fifo_path, 'r') as fifo:
            value = fifo.readline().strip()
            if value:
                print(f"Received: {value}")
                if value in ['F', 'B', 'L', 'R']:
                    direction_counter[value] += 1  
                    write_most_common_direction() 
                if not send_data(value):
                    print('Resending data...')
                    time.sleep(0.5) 

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        ser.write("Q".encode())
        print("Exiting...")
    finally:
        ser.close()
