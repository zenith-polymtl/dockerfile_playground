const terminal = document.getElementById('terminal');
const maxTerminalLines = 100;
let terminalLines = 0;

function addTerminalOutput(message) {
    const line = document.createElement('div');
    line.innerHTML = message;
    terminal.appendChild(line);
    
    terminalLines++;
    if (terminalLines > maxTerminalLines) {
        terminal.removeChild(terminal.firstChild);
        terminalLines--;
    }
    
    terminal.scrollTop = terminal.scrollHeight;
}

addTerminalOutput('<span class="timestamp">[SYSTEM]</span> Connecting to drone control...');

const ws = new WebSocket(`ws://${window.location.host}/ws`);

ws.onopen = () => {
    addTerminalOutput('<span class="timestamp">[SYSTEM]</span> Connection established');
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.type === 'sensor_data') {
        document.getElementById('waterQty').textContent = `Water Qty: ${data.data.water_qty} mL`;
        document.getElementById('torque').textContent = `Torque: ${data.data.torque.toFixed(2)} Nm`;
    }
    else if (data.type === 'terminal') {
        addTerminalOutput(data.message);
    }
};

ws.onclose = () => {
    addTerminalOutput('<span class="timestamp">[SYSTEM]</span> Connection closed');
};

ws.onerror = (error) => {
    addTerminalOutput(`<span class="timestamp">[ERROR]</span> ${error.message}`);
};
function sendCommand(command, data='') {
  ws.send(JSON.stringify({ 
    type: 'command', 
    command: command,
    data: data
  }));
}

function setAmount(id, command, data='') {
    const amount = document.getElementById(id).value;
    if (amount) {
        ws.send(JSON.stringify({ 
            type: 'command', 
            command,
            data: data,
            amount: amount 
        }));
    }
}

function selectRadioButton(button) {
  const parent = button.parentElement;
  [...parent.children].forEach(sib => sib.classList.remove('selected'));

  button.classList.add('selected');

  const command = button.getAttribute('data-command');
  const value = button.getAttribute('data-value');
  sendCommand(command, value);
}