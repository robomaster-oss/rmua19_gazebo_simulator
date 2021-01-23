from flask import Flask, render_template, request
from flask_cors import CORS
from flask_socketio import SocketIO, emit


app = Flask(__name__)
app.config['SECRET_KEY'] = 'randonlygeneratedkeys'
CORS(app)
socketio = SocketIO(app)


@app.route("/",  methods=['GET','POST'])
def login():
    return render_template('gamepad.html')

@socketio.on_error_default
def default_error_handler(e):
    print("======================= ERROR")
    print(e)
    print(request.event["message"])
    print(request.event["args"])
    print("======================= ")
    

@socketio.on('control', namespace='/control')
def control_message(message):
    gp_data=message['gp_data']
    print("msg len",len(gp_data))
    print("msg",message)


if __name__ == "__main__":
    # socketio.run(app)
    socketio.run(app,debug = True, host="0.0.0.0", port = 5000)
   