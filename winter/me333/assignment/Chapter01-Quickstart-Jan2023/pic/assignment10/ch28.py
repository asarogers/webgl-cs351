import serial
import matplotlib.pyplot as plt

# Open serial connection
ser = serial.Serial('/dev/ttyUSB0', 230400)
print('Opening port:', ser.name)

has_quit = False

# Menu loop
while not has_quit:
    print('\nPIC32 MOTOR DRIVER INTERFACE')
    print('\ta: Read ADC (raw counts)\tb: Read current sensor (mA)')
    print('\tc: Read encoder count\td: Read encoder degrees\te: Reset encoder')
    print('\tf: Set PWM (-100 to 100)\tp: Unpower motor')
    print('\tg: Set current gains\th: Get current gains')
    print('\tr: Get mode\t\tk: Run ITEST mode')
    print('\ti: Set position gains\tj: Get position gains')
    print('\tl: Go to angle (deg)\tq: Quit')

    # Get user command
    selection = input('\nENTER COMMAND: ')
    selection_endline = selection + '\n'

    # Send command to PIC32
    ser.write(selection_endline.encode())

    # Handle user selection
    if selection == 'a':  # Read ADC (raw counts)
        response = ser.read_until(b'\n').decode().strip()
        print(f'ADC Value: {response}')
        
    elif selection == 'b':  # Read current (mA)
        response = ser.read_until(b'\n').decode().strip()
        print(f'Current (mA): {response}')
        
    elif selection == 'c':  # Read encoder count
        response = ser.read_until(b'\n').decode().strip()
        print(f'Encoder Count: {response}')
        
    elif selection == 'd':  # Read encoder degrees
        response = ser.read_until(b'\n').decode().strip()
        print(f'Encoder Degrees: {response}')
        
    elif selection == 'e':  # Reset encoder
        print('Encoder reset command sent.')
        
    elif selection == 'f':  # Set PWM (-100 to 100)
        pwm_value = input('Enter PWM (-100 to 100): ')
        ser.write((pwm_value + '\n').encode())
        print(f'PWM set to {pwm_value}')

    elif selection == 'g':  # Set current gains
        kp = input('Enter kp_mA: ')
        ki = input('Enter ki_mA: ')
        ser.write(f"{kp} {ki}\n".encode())
        print(f'Current gains set: kp={kp}, ki={ki}')
        
    elif selection == 'h':  # Get current gains
        response = ser.read_until(b'\n').decode().strip()
        print(f'Current gains (kp, ki): {response}')

    elif selection == 'i':  # Set position gains
        kp = input('Enter kp_deg: ')
        ki = input('Enter ki_deg: ')
        kd = input('Enter kd_deg: ')
        ser.write(f"{kp} {ki} {kd}\n".encode())
        print(f'Position gains set: kp={kp}, ki={ki}, kd={kd}')

    elif selection == 'j':  # Get position gains
        response = ser.read_until(b'\n').decode().strip()
        print(f'DEBUG: Received position gains: {response}')
        
    elif selection == 'k':  # Run ITEST mode
        print('ITEST mode running...')
        data_length = int(ser.read_until(b'\n').decode().strip())  # Read length
        ref_values = []
        actual_values = []

        for i in range(data_length):
            response = ser.read_until(b'\n').decode().strip()
            ref, actual = map(int, response.split())
            ref_values.append(ref)
            actual_values.append(actual)

        # Plot the data
        plt.figure(figsize=(8, 4))
        plt.plot(ref_values, label='Reference Current (mA)', linestyle='--', color='r')
        plt.plot(actual_values, label='Measured Current (mA)', linestyle='-', color='b')
        plt.xlabel('Sample Index')
        plt.ylabel('Current (mA)')
        plt.title('ITEST Mode: Reference vs Measured Current')
        plt.legend()
        plt.grid()
        plt.show()
    
    elif selection == 'l':  # Set target position (Go to angle)
        angle = input('Enter desired angle (degrees): ')
        ser.write((angle + '\n').encode())

        # Read confirmation from PIC32
        response = ser.read_until(b'\n').decode().strip()
        print(f'PIC32 Response: {response}')
        
    elif selection == 'p':  # Unpower motor (set mode to IDLE)
        print('Motor unpowered (IDLE mode).')

    elif selection == 'q':  # Quit the program
        print('Exiting client.')
        has_quit = True
        ser.close()

    elif selection == 'r':  # Get mode
        response = ser.read_until(b'\n').decode().strip()
        print(f'Current mode: {response}')

    elif selection == 'x':  # Debug HOLD Mode
        print('Collecting HOLD mode data...')
        data_length = int(ser.read_until(b'\n').decode().strip())  # Read length
        ref_values = []
        actual_values = []

        for _ in range(data_length):
            response = ser.read_until(b'\n').decode().strip()
            
            if "DEBUG" in response or not response.replace(" ", "").replace(".", "").isdigit():
                print(f"Skipping non-numeric line: {response}")
                continue
            
            ref, actual = map(float, response.split())
            ref_values.append(ref)
            actual_values.append(actual)

        # Plot the data
        plt.figure(figsize=(8, 4))
        plt.plot(ref_values, label='Reference Current (mA)', linestyle='--', color='r')
        plt.plot(actual_values, label='Measured Current (mA)', linestyle='-', color='b')
        plt.xlabel('Sample Index')
        plt.ylabel('Current (mA)')
        plt.title('HOLD Mode: Reference vs Measured')
        plt.legend()
        plt.grid()
        plt.show()

    else:
        print(f'Invalid selection: {selection}')
