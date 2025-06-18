from flask import Flask, request, jsonify, render_template_string
import random

app = Flask(__name__)

secret_number = random.randint(1, 100)

HTML_PAGE = """
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8" />
    <title>猜数字小游戏</title>
</head>
<body>
    <h1>猜数字小游戏（1-100）</h1>
    <input type="number" id="guessInput" placeholder="输入你的猜测" />
    <button onclick="makeGuess()">猜！</button>
    <p id="result"></p>

    <script>
        async function makeGuess() {
            const guess = document.getElementById('guessInput').value;
            if (!guess) {
                alert('请输入一个数字');
                return;
            }
            const response = await fetch('/guess', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({guess: guess})
            });
            const data = await response.json();
            const resultP = document.getElementById('result');
            if (data.result === 'correct') {
                resultP.textContent = '恭喜你，猜对了！游戏将重新开始。';
            } else if (data.result === 'too low') {
                resultP.textContent = '太小了，再试试！';
            } else {
                resultP.textContent = '太大了，再试试！';
            }
        }
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/guess', methods=['POST'])
def guess():
    global secret_number
    data = request.json
    guess = int(data.get('guess'))
    if guess == secret_number:
        response = {'result': 'correct'}
        secret_number = random.randint(1, 100)
    elif guess < secret_number:
        response = {'result': 'too low'}
    else:
        response = {'result': 'too high'}
    return jsonify(response)

if __name__ == '__main__':
    app.run(debug=True)
