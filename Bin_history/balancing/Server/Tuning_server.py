from flask import Flask, request, render_template_string
import serial
import time

BT_SERIAL_PORT = 'COM5'  # Change to your Bluetooth serial port
BAUD_RATE = 115200

app = Flask(__name__)

# Default values for form fields
default_values = { 
    'kp': '85',
    'kd': '0.9',
    'kp_o': '8',
    'kd_o': '0.9',
    'kp_v': '0',
    'kd_v': '0',
    'kv': '200',
    'bias': '0.166'
}

html_template = '''
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 PID Tuner</title>
    <style>
        body { font-family: Arial; background: #f4f4f4; padding: 40px; }
        .container { background: #fff; padding: 30px; max-width: 500px; margin: auto; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.2);}
        h2 { text-align: center; }
        input[type=number] { width: 100%; padding: 10px; margin: 6px 0 12px; border: 1px solid #ccc; border-radius: 4px; }
        input[type=submit] { background: #28a745; color: white; padding: 12px; width: 100%; border: none; border-radius: 4px; font-size: 16px; cursor: pointer; }
        input[type=submit]:hover { background: #218838; }
    </style>
</head>
<body>
    <div class="container">
        <h2>ESP32 PID Tuning Interface</h2>
        <form method="post">
            <label>Inner Loop (Gyro)</label>
            Kp: <input type="number" name="kp" step="0.01" value="{{ values.kp }}" required>
            Kd: <input type="number" name="kd" step="0.01" value="{{ values.kd }}" required>

            <label>Middle Loop (Tilt)</label>
            Kp_o: <input type="number" name="kp_o" step="0.01" value="{{ values.kp_o }}" required>
            Kd_o: <input type="number" name="kd_o" step="0.01" value="{{ values.kd_o }}" required>

            <label>Outer Loop (Velocity)</label>
            Kp_v: <input type="number" name="kp_v" step="0.01" value="{{ values.kp_v }}" required>
            Kd_v: <input type="number" name="kd_v" step="0.01" value="{{ values.kd_v }}" required>

            <label>kv:</label>
            <input type="number" name="kv" step="0.1" value="{{ values.kv }}" required>

            <label>Tilt Bias:</label>
            <input type="number" name="bias" step="0.01" value="{{ values.bias }}" required>

            <input type="submit" value="Send to ESP32">
        </form>
        {% if sent %}
        <p style="color: green; margin-top: 20px;">Sent: {{ message }}</p>
        {% endif %}
    </div>
</body>
</html>
'''

def send_to_esp(message):
    try:
        with serial.Serial(BT_SERIAL_PORT, BAUD_RATE, timeout=2) as ser:
            time.sleep(1)
            ser.write(message.encode())
            print(f"Sent to ESP: {message.strip()}")
    except serial.SerialException as e:
        print(f"Serial Error: {e}")

@app.route('/', methods=['GET', 'POST'])
def index():
    values = default_values.copy()
    sent = False
    message = ""

    if request.method == 'POST':
        # Update current values from form
        for key in values.keys():
            values[key] = request.form.get(key, values[key])

        # Build message string to send to ESP
        message = f"{values['kp']} {values['kd']} {values['kp_o']} {values['kd_o']} " \
                  f"{values['kp_v']} {values['kd_v']} {values['kv']} {values['bias']}\n"
        send_to_esp(message)
        sent = True

    return render_template_string(html_template, values=values, sent=sent, message=message)

if __name__ == '__main__':
    app.run(debug=True)
