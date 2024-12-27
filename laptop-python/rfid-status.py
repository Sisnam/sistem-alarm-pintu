import serial

ser = serial.Serial(
    port='COM6',    # Ganti dengan port yang benar
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

print("Menunggu data dari ATXmega melalui Arduino Mega...")

try:
    while True:
        if ser.in_waiting > 0:  # Jika ada data di serial buffer
            data = ser.readline().decode('utf-8').strip()
            print("Data diterima:", data)

except KeyboardInterrupt:
    print("Program dihentikan.")
finally:
    ser.close()
