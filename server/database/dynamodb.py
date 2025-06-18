# implement DynamoDB CRUD operations

from aws_config import dynamodb
import boto3
from boto3.dynamodb.conditions import Key
from decimal import Decimal
from datetime import datetime
from .aws_config import AWS_REGION
import botocore.exceptions


dynamodb = boto3.resource("dynamodb", region_name="eu-north-1")
table = dynamodb.Table("KeyLocations")

_table_cache = {}

def get_table(name):
    if name not in _table_cache:
        _table_cache[name] = dynamodb.Table(name)
    return _table_cache[name]


def update_robot_state(device_id, x, y, battery, mode, status, current_key_loc):
    table = get_table('RobotState')
    from .utils import iso_timestamp
    table.put_item(Item={
        'device_id': device_id,
        'timestamp': iso_timestamp(),
        'x': Decimal(str(x)),
        'y': Decimal(str(y)),
        'battery': Decimal(str(battery)),
        'mode': mode,
        'status': status,
        'current_key_loc': current_key_loc
    })

def log_location(device_id, x, y):
    table = get_table('LocationHistory')
    from .utils import iso_timestamp
    table.put_item(Item={
        'device_id': device_id,
        'timestamp': iso_timestamp(),
        'x': Decimal(str(x)),
        'y': Decimal(str(y))
    })

def save_key_location(name, x, y, theta=0.0):
    item = {
        "name": name,
        "x": Decimal(str(x)),
        "y": Decimal(str(y)),
        "theta": Decimal(str(theta)),
        "created_at": datetime.utcnow().isoformat()
    }
    table.put_item(Item=item)
    print(f"[DynamoDB] Stored location: {item}")

def update_pid_values(device_id, loop, pg, dg, ig, sp, rot=0):
    table = get_table('PIDConfigs')
    table.put_item(Item={
        'device_id': device_id,
        'loop': loop,
        'pg': Decimal(str(pg)),
        'dg': Decimal(str(dg)),
        'ig': Decimal(str(ig)),
        'sp': Decimal(str(sp)),
        'rot': Decimal(str(rot))
    })

def get_all_key_locations():
    table = get_table('KeyLocations')
    response = table.scan()
    return response['Items']

def find_nearest_key_location(x, y):
    from math import sqrt
    key_locations = get_all_key_locations()
    closest = None
    min_dist = float('inf')
    for loc in key_locations:
        dx = float(loc['x']) - x
        dy = float(loc['y']) - y
        dist = sqrt(dx**2 + dy**2)
        if dist < min_dist:
            min_dist = dist
            closest = loc['name']
    return closest or "Unknown"

def get_battery_level():
    try:
        with open('/sys/class/power_supply/BAT0/capacity', 'r') as f:
            return float(f.read().strip())
    except:
        return 100.0  # default fallback

def get_mode():
    return "AUTONOMOUS"  # TODO: hook to actual mode topic or flag

def get_status(velocity):
    return "MOVING" if velocity > 0.1 else "IDLE"


def safe_put_item(table, item):
    try:
        table.put_item(Item=item)
    except botocore.exceptions.BotoCoreError as e:
        print(f"Failed to write to {table.name}: {e}")

# In on_connect() in mqtt_serial_bridge.py, add this line:
# client.subscribe("robot/auto/key/assign")

# on_message() in mqtt_serial_bridge.py, add this block:
"""elif topic == "robot/auto/key/assign":
    try:
        data = json.loads(payload)
        x = float(data['x'])
        y = float(data['y'])
        theta = float(data.get('theta', 0.0))
        name = data.get('name', f"Location_{int(x)}_{int(y)}")  # fallback name
        from dynamodb import save_key_location
        save_key_location(name, x, y, theta)
    except Exception as e:
        print(f"[MQTT] Error handling key location assign: {e}")
"""
# Update Frontend mqtt-control.js
# Make sure when the "Assign" button is clicked, it sends:
"""
const payload = JSON.stringify({
  name: "DockingStation",  // add this if not already
  x: parseFloat(document.getElementById('key-x')?.value),
  y: parseFloat(document.getElementById('key-y')?.value),
  theta: 0.0
});
pub('robot/auto/key/assign', payload);
"""