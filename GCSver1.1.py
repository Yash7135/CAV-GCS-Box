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


old_servoopen = 1 #payload dropping opening and arming
old_servoclose = 1 # payload dropping disarming and closing 
     
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
            servoopen = float(input_list[9])
            servoclose = float(input_list[10])

            safetyon=int(input_list[11])

            tilt = float(input_list[12])
            pan = float(input_list[13])
            zoom_value=float(input_list[14])

    #----------------------------------------------------------------
            if (safetyoff != old_safetyoff):
                if (safetyoff == 0):
                    vehicle.mav.command_long_send(
                        vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                        0,
                        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                        0, #0=safetyoff
                        0, 
                        0, 
                        0, 
                        0, 
                        0)
                old_safetyoff = safetyoff

            #--------------------------------------------------------------
            if (arm != old_arm):
                if (arm == 0):
                    armdisarm = dialect.MAVLink_command_long_message(
                            target_system=vehicle.target_system,
                            target_component=vehicle.target_component,
                            command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
                            confirmation=0,
                            param1=1,
                            param2=0,  
                            param3=0,
                            param4=0,
                            param5=0,
                            param6=0,
                            param7=0
                            )
                    # Send the ARM/DISARM command
                    vehicle.mav.send(armdisarm)
                old_arm = arm

            #----------------------------------------------------------------
            if (disarm != old_disarm):
                if (disarm == 0):
                    armdisarm = dialect.MAVLink_command_long_message(
                                target_system=vehicle.target_system,
                                target_component=vehicle.target_component,
                                command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
                                confirmation=0,
                                param1=0,
                                param2=0,  
                                param3=0,
                                param4=0,
                                param5=0,
                                param6=0,
                                param7=0
                                )
                        # Send the ARM/DISARM command
                    vehicle.mav.send(armdisarm)
                old_disarm = disarm

            #----------------------------------------------------------------
            if (alt_hold != old_alt_hold):
                if (alt_hold == 0):
                    set_mode_message = dialect.MAVLink_command_long_message(
                            target_system=vehicle.target_system,
                            target_component=vehicle.target_component,
                            command=dialect.MAV_CMD_DO_SET_MODE,
                            confirmation=0,
                            param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            param2=2,
                            param3=0,
                            param4=0,
                            param5=0,
                            param6=0,
                            param7=0
                            )
                            # change flight mode
                    vehicle.mav.send(set_mode_message)
                old_alt_hold = alt_hold

            #----------------------------------------------------------------   
            if (loiter != old_loiter):
                if (loiter == 0):

                    set_mode_message = dialect.MAVLink_command_long_message(
                            target_system=vehicle.target_system,
                            target_component=vehicle.target_component,
                            command=dialect.MAV_CMD_DO_SET_MODE,
                            confirmation=0,
                            param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            param2=5,
                            param3=0,
                            param4=0,
                            param5=0,
                            param6=0,
                            param7=0
                            )
                            # change flight mode
                    vehicle.mav.send(set_mode_message)
                old_loiter = loiter
            #---------------------------------------------------------------- 
            if (auto != old_auto):
                if (auto == 0):
                    set_mode_message = dialect.MAVLink_command_long_message(
                            target_system=vehicle.target_system,
                            target_component=vehicle.target_component,
                            command=dialect.MAV_CMD_DO_SET_MODE,
                            confirmation=0,
                            param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            param2=3,
                            param3=0,
                            param4=0,
                            param5=0,
                            param6=0,
                            param7=0
                            )
                            # change flight mode
                    vehicle.mav.send(set_mode_message)  
                old_auto = auto
            #----------------------------------------------------------------  
            if (rtl != old_rtl):
                if (rtl == 0):
                    set_mode_message = dialect.MAVLink_command_long_message(
                            target_system=vehicle.target_system,
                            target_component=vehicle.target_component,
                            command=dialect.MAV_CMD_DO_SET_MODE,
                            confirmation=0,
                            param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            param2=6,
                            param3=0,
                            param4=0,
                            param5=0,
                            param6=0,
                            param7=0
                            )
                            # change flight mode
                    vehicle.mav.send(set_mode_message) 
                old_rtl = rtl
            #----------------------------------------------------------------   
            if (land != old_land):
                if (land == 0):
                    set_mode_message = dialect.MAVLink_command_long_message(
                                    target_system=vehicle.target_system,
                                    target_component=vehicle.target_component,
                                    command=dialect.MAV_CMD_DO_SET_MODE,
                                    confirmation=0,
                                    param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                    param2=9,
                                    param3=0,
                                    param4=0,
                                    param5=0,
                                    param6=0,
                                    param7=0
                                    )
                                    # change flight mode
                    vehicle.mav.send(set_mode_message) 
                old_land = land


            #----------------------------------------------------------------   
            
            if (camera_auto_center == 0):
                vehicle.mav.command_long_send(
                vehicle.target_system,
                vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                0,  # confirmation
                0,  # reset tilt angle
                0,  # reset pan angle
                0,  #
                0, 
                0, 0, 0)
            else:
                if (tilt!=old_tilt or pan!=old_pan):
                    vehicle.mav.command_long_send(
                    vehicle.target_system,
                    vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                    0,  # confirmation
                    float('nan'),  # tilt angle in centidegrees
                    float('nan'),   # pan angle in centidegrees
                    tilt, pan, 0, 0, 0)
            old_tilt=tilt
            old_pan=pan

            #---------------------------------------------------------------- 
 
            if abs(zoom_value - old_zoom_value) > 2:
                vehicle.mav.command_long_send(
                            vehicle.target_system,
                            vehicle.target_component,
                            mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM,
                            0,  # confirmation
                            2,  # absolute control
                            zoom_value,  # Zoom out
                            0,  
                            0, 
                            0, 0, 0)
                old_zoom_value = zoom_value
            #----------------------------------------------------------------   
            if (servoopen != old_servoopen):
                if (servoopen == 0):
                    vehicle.mav.command_long_send(
                        vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0,
                        9,
                        1850, 
                        0, 
                        0, 
                        0, 
                        0, 
                        0)
                    vehicle.mav.command_long_send(
                        vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0,
                        11,
                        1850, 
                        0, 
                        0, 
                        0, 
                        0, 
                        0)
                    vehicle.mav.command_long_send(
                        vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0,
                        13,
                        1850, 
                        0, 
                        0, 
                        0, 
                        0, 
                        0)         
                old_servoopen = servoopen
            #----------------------------------------------------------------   
            if (servoclose != old_servoclose):
                if (servoclose == 0):
                    vehicle.mav.command_long_send(
                        vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0,
                        9,
                        1250, 
                        0, 
                        0, 
                        0, 
                        0, 
                        0)
                    vehicle.mav.command_long_send(
                        vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0,
                        11,
                        1250, 
                        0, 
                        0, 
                        0, 
                        0, 
                        0)
                    vehicle.mav.command_long_send(
                        vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0,
                        13,
                        1250, 
                        0, 
                        0, 
                        0, 
                        0, 
                        0)         

                old_servoclose = servoclose
            #----------------------------------------------------------------   
            if (safetyon != old_safetyon):
                if (safetyon == 0):
                    vehicle.mav.command_long_send(
                        vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                        0,
                        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                        1, 
                        0, 
                        0, 
                        0, 
                        0, 
                        0)
                old_safetyon = safetyon

    except serial.serialutil.SerialException as e:
        print("Error", e)
        print(f"Serial exception occurred: {e}")
        print("Reinitializing serial connection...")
        arduino = serial_connection()

    except ValueError as e:
        print(f"Error processing command: {command}, Error: {e}")