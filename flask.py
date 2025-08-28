from flask import Flask

app = Flask(Telemetry)

@app.route("/")
def hello_world():
    return "<p>Hello, World!</p>"