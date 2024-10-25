import os
import time
import socket
import BlynkLib
from BlynkTimer import BlynkTimer
import psutil

BLYNK_AUTH_TOKEN = '###'
blynk = BlynkLib.Blynk(BLYNK_AUTH_TOKEN)
timer = BlynkTimer()

@blynk.on("connected")
def blynk_connected():
    print("Hi, You have Connected to New Blynk2.0")
    time.sleep(2)

def get_wifi_info():
    try:
        ssid = os.popen("iwgetid -r").read().strip()
        hostname = socket.gethostname()
        ip_address = socket.gethostbyname(hostname)
        return ssid, ip_address
    except Exception as e:
        print(f"Error getting WiFi information: {e}")
        return None, None

def get_cpu_temperature():
    try:
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as temp_file:
            temp = float(temp_file.readline().strip()) / 1000  # Convert from millidegrees to degrees
            return temp
    except Exception as e:
        print(f"Error reading CPU temperature: {e}")
        return None

def get_system_info():
    temperature = get_cpu_temperature()
    if temperature is not None:
        blynk.virtual_write(1, temperature)

    memory = psutil.virtual_memory()
    memory_usage = memory.percent
    disk = psutil.disk_usage('/')
    disk_usage = disk.percent
    cpu_usage = psutil.cpu_percent(interval=1)
    ssid, ip_address = get_wifi_info()
    if ssid and ip_address:
        blynk.virtual_write(8, ssid)
        blynk.virtual_write(9, ip_address)

    blynk.virtual_write(4, memory_usage)
    blynk.virtual_write(5, disk_usage)
    blynk.virtual_write(6, cpu_usage)
def read_data():
    try:
        with open('/home/pasindu/newProject/msm.txt', 'r') as file:
            lines = file.readlines()
            if len(lines) >= 3:
                direction = lines[0].split(": -")[-1].strip()
                voltage = lines[1].split(": -")[-1].strip()
                ldr = lines[2].split(": -")[-1].strip()

                blynk.virtual_write(2, direction)
                blynk.virtual_write(3, voltage)
                blynk.virtual_write(7, ldr)
    except Exception as e:
        print(f"Error reading data: {e}")

timer.set_interval(2, read_data)
timer.set_interval(5, get_system_info)

while True:
    blynk.run()
    timer.run()