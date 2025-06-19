# import serial
import json
import os
import csv

# === CONFIGURATION ===
SERIAL_PORT = '/dev/ttyUSB0'      # Change to your actual serial port
BAUD_RATE = 115200                # Match the baud rate with your device
CSV_FILE = 'battery_log.csv'
# ======================
E_max = 92289
Ev_last = E_max
Ei_last = E_max
Et_last = E_max
Percentage = 1.0
# ======================

def initialize_csv_file():
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["VB", "EU"])

def append_to_csv(VB, EU):
    with open(CSV_FILE, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([VB, EU])

def calculate_percentage(VB: float, EU: float) -> int:
    global Ev_last, Et_last, Percentage

    # 1) Map VB → Ev using your piecewise formula
    if VB > 14.8:
        Ev = 13009 * VB - 115094
    elif VB > 14.2:
        Ev = 96166 * VB - 1346076
    elif VB > 12.77:
        Ev = 12606 * VB - 159533
    else:
        Ev = 314 * VB - 2460

    # 2) Never let Ev increase from its previous value
    if Ev > Ev_last:
        Ev = Ev_last
    Ev_last = Ev

    # 3) Compute Ei from last total-energy and cumulative EU
    Ei = Et_last - EU

    # 4) Blend Ei and Ev into new Et based on current Percentage
    pct_val = Percentage * 100
    if 22 < pct_val < 84:
        Et = 0.7 * Ei + 0.3 * Ev
    else:
        Et = 0.3 * Ei + 0.7 * Ev

    Et_last = Et

    # 5) Compute new Percentage fraction
    Percentage = Et / E_max

    # 6) Return as integer percent
    return int(Percentage * 100), Et
