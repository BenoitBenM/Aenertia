// Client-side JavaScript to handle UI interactions and communicate with the backend
// - Displays alerts
// - Triggers hardware actions (e.g. flash LED)
// - Manages tab switching with unsaved changes warning
// - Loads and displays stored PID values in tables


function sayHello() {
  alert("Hi there! This is a JS alert.");
}

// Send request to server to flash LED and show the result in an alert
function flashLED() {
  fetch('/flash-led')
    .then(response => response.text())
    .then(data => alert(data))
    .catch(error => alert("Failed to flash LED: " + error));
}

// Track the currently active tab and whether the form has unsaved changes
let currentTab = 'pidTab';
let isDirty = false;

// Mark page as "dirty" when any input is modified (used to warn before switching tabs)
document.addEventListener('input', () => {
  isDirty = true;
});

function showTab(tabId) {
  if (tabId === currentTab) return;

  if (isDirty && !confirm("You have unsaved changes. Are you sure you want to switch tabs?")) {
    return;
  }

  document.querySelectorAll('.tab-content').forEach(tab => tab.classList.remove('active'));
  document.getElementById(tabId).classList.add('active');
  currentTab = tabId;
  isDirty = false;
}

// Populate PID tables with previously submitted values
// Fetch and display saved PID values in inner and outer tables
function populatePIDTables() {
  fetch('/pid-values')
    .then(res => res.json())
    .then(data => {
      ['inner', 'outer'].forEach(loop => {
        const table = document.getElementById(loop + 'Table');
        data[loop].forEach(rowValues => {
          const row = table.insertRow(-1);
          rowValues.forEach(val => row.insertCell().textContent = val);
        });
      });
    });
}

// Call populate on page load
// Populate PID tables when the page finishes loading
document.addEventListener('DOMContentLoaded', () => {
  populatePIDTables();
});

function startMove(direction) {
  if (client.connected) {
    client.publish('robot/serial', direction);
  }
}

function stopMove() {
  if (client.connected) {
    client.publish('robot/serial', 'STOP');
  }
}

