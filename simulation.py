from serial import Serial
import time
import serial
import pymavlink.mavutil as utility
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect

def serial_connection():
    while True:
        try:
            arduino = Serial(port='COM68', baudrate=9600, timeout=0.1)
            print("Serial connection to Arduino established.")
            return arduino
        except serial.serialutil.SerialException as e:
            print(f"Failed to open serial port: {e}")
            print("Retrying in 5 seconds...")
            time.sleep(5)


arduino = serial_connection()

# Create a MAVLink connection to the vehicle
vehicle = utility.mavlink_connection('udpin:127.0.0.1:14561')

# Wait for a heartbeat from the vehicle to establish communication
vehicle.wait_heartbeat()

# Inform the user of the connection

print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

old_safetyon= 1
old_safetyoff= 1

old_arm=1
old_disarm=1

old_alt_hold=1
old_loiter=1
old_auto=1
old_rtl=1
old_land=1  

old_zoom_value = 0


old_tilt = 1
old_pan = 1


old_scr_user1AO = 1 #payload dropping opening and arming
old_scr_user2DC = 1 # payload dropping disarming and closing 

old_scr_user3Safe = 1 #payload safety 


while True:
    try:
        command = arduino.readline().strip()
        if command:
            command = command.decode('utf-8')
            input_list=command.split(",")
            print(input_list)  
    
    #----------------------------------------------------------------
            safetyoff = int(input_list[0])
            arm = int(input_list[1])
            disarm = int(input_list[2])

            alt_hold=int(input_list[3])
            loiter=int(input_list[4])
            auto=int(input_list[5])
            rtl=int(input_list[6])
            land=int(input_list[7])

            camera_auto_center=int(input_list[8])
            scr_user1AO = float(input_list[9])
            scr_user2DC = float(input_list[10])


            safetyon=int(input_list[11])

            tilt = float(input_list[12])
            pan = float(input_list[13])
            zoom_value=float(input_list[14])
           
            scr_user3Safe = float(input_list[15])
            #---------------------------------------------------------------- 
            if (scr_user1AO != old_scr_user1AO):
                if (scr_user1AO == 0):
                    vehicle.mav.param_set_send(
                    target_system=vehicle.target_system,
                    target_component=vehicle.target_component,
                    param_id=b'SCR_USER1',
                    param_value=1,
                    param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32
                )
                old_scr_user1AO = scr_user1AO
            #----------------------------------------------------------------   
            if (scr_user2DC != old_scr_user2DC):
                if (scr_user2DC == 0):
                    vehicle.mav.param_set_send(
                    target_system=vehicle.target_system,
                    target_component=vehicle.target_component,
                    param_id=b'SCR_USER2',
                    param_value=1,
                    param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32
                )
                    
                old_scr_user2DC = scr_user2DC
            #----------------------------------------------------------------  
            if (scr_user3Safe != old_scr_user3Safe):
                if (scr_user3Safe == 1):
                    vehicle.mav.param_set_send(
                        target_system=vehicle.target_system,
                        target_component=vehicle.target_component,
                        param_id=b'SCR_USER3',
                        param_value=1,
                        param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32
                    )      
                old_scr_user3Safe = scr_user3Safe

            #----------------------------------------------------------------  
    except serial.serialutil.SerialException as e:
        print("Error", e)
        print(f"Serial exception occurred: {e}")
        print("Reinitializing serial connection...")
        arduino = serial_connection()

    except ValueError as e:
        print(f"Error processing command: {command}, Error: {e}")