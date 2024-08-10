import serial
import matplotlib.pyplot as plt
import numpy as np

# Replace with your serial port and baud rate
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

def parse_fft_data(line):
    try:
        data = line.strip().decode('utf-8')
        if data.startswith('(') and data.endswith(')'):
            # Split by commas, remove parentheses, and convert to float
            data = data[1:-1].split('),(')
            frequencies = []
            magnitudes = []
            for point in data:
                freq, mag = map(float, point.split(','))
                frequencies.append(freq)
                magnitudes.append(mag)
            return frequencies, magnitudes
    except Exception as e:
        print(f"Error parsing data: {e}")
    return [], []

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    plt.ion()
    fig, ax = plt.subplots()
    bars = None
    maxMag = 0


    try:
        while True:
            line = ser.readline()
            frequencies, magnitudes = parse_fft_data(line)

            if frequencies and magnitudes:
                if bars is None:
                    bars = ax.bar(frequencies, magnitudes, width=2)
                    ax.set_xlim(min(frequencies), max(frequencies))
                    ax.set_ylim(0, 8500)
                    plt.xlabel('Frequency (Hz)')
                    plt.ylabel('Magnitude')
                    plt.title('Real-time FFT Data')
                else:
                    for bar, mag in zip(bars, magnitudes):
                        bar.set_height(mag)
                        if mag > maxMag:
                            maxMag = mag
                            ax.set_ylim(0, maxMag)
                ax.set_xticks(frequencies)
                ax.set_xticklabels([f"{int(freq)}" for freq in frequencies], rotation=45)
                plt.pause(0.01)  # Adjust the pause time as needed
    finally:
        print("max magnitude:")
        print(maxMag)

if __name__ == "__main__":
    main()

