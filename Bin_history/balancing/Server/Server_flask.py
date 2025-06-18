from flask import Flask, render_template_string, request
import serial

app = Flask(__name__)

SERIAL_PORT = '/dev/ttyS0'
BAUDRATE = 9600

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>PID Controller Settings</title>
</head>
<body>
    <h2>Set PID parameters and setpoint</h2>
    <form method="POST" action="/">
        <label for="kp">Kp:</label><br>
        <input type="number" step="any" id="kp" name="kp" required><br>
        <label for="ki">Ki:</label><br>
        <input type="number" step="any" id="ki" name="ki" required><br>
        <label for="kd">Kd:</label><br>
        <input type="number" step="any" id="kd" name="kd" required><br>
        <label for="setpoint">Setpoint:</label><br>
        <input type="number" step="any" id="setpoint" name="setpoint" required><br><br>
        <input type="submit" value="Send">
    </form>
    {% if message %}
    <p>{{ message }}</p>
    {% endif %}
</body>
</html>
'''

@app.route('/', methods=['GET', 'POST'])
def index():
    message = ""
    if request.method == 'POST':
        try:
            kp = float(request.form['kp'])
            ki = float(request.form['ki'])
            kd = float(request.form['kd'])
            setpoint = float(request.form['setpoint'])

            data_str = f"{kp} {ki} {kd} {setpoint}\n"
            print(data_str)
            
            # Open, write, and close serial port every POST to avoid locking issues
            with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
                ser.write(data_str.encode('utf-8'))

            message = "Values sent successfully!"
        except Exception as e:
            message = f"Error: {e}"

    return render_template_string(HTML_TEMPLATE, message=message)

if __name__ == '__main__':
    # Run without debug or reload to avoid locking issues
    app.run(host='0.0.0.0', port=5000, debug=False)
