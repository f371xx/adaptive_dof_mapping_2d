"""Starts a simple python server to run the simulation"""
import os
import time
import csv
import json
import socketio
import eventlet
import numpy as np
from engineio.payload import Payload
Payload.max_decode_packets = 500
import SimModel2D

# General Server information
ADDRESS, PORT = 'localhost', 4000
LOG_PATH = "./logs/" # Store path of recordings

# Server provided files
FILE_PATH = "./WebsiteFiles/"
HTML_FILE_0 = "/", 'text/html'
FILES = [
    ("/Simulation.html", 'text/html'),
    ("/general.js", 'text/javascript'),
    ("/favicon.ico", 'image/x-icon'),
    ("/2DSim.js", 'text/javascript'),
    ("/box2dweb/Box2d.min.js",'text/javascript'),
    ("/drawFunctions.js",'text/javascript'),
    ("/Controls.html", 'text/html'),
    ("/Controller.png", 'image/png'),
    ("/style.css", 'text/css'),
]
static_files = {
    file[0]: {'filename':FILE_PATH+file[0],'content_type':file[1]} for file in FILES}
static_files[HTML_FILE_0[0]]= {'filename':FILE_PATH+FILES[0][0],
                               'content_type':'text/html'} # manually add standard

# IOSocket events
MODEL_NAMES = "modelNames"
DOF_REQUEST = "dofs"
DOF_REQUEST_MODELNAME="model"
DOF_REQUEST_STATE="state"
ABOUT_REQUEST = "about"
START_RECORDING = "StartRecording"
STORE_STATE = "StoreState"
STOP_RECORDING = "StopRecording"
UPDATE_MODELS = "UpdateServerModels"
PRINT_STATELOGS = "PrintStateLogs"
CLEAR_STATELOGS = "ClearStateLogs"
GET_MODEL_VECTORS = "GetModelVectors"
RESTARTING_SIM = "RestartingSimulation"
STORE_COMPLETE_REPLAY = "StoreCompleteReplay"


def loadModels():
    """Loads all models from files"""
    global model_names, models
    print('Loading models')
    model_names = SimModel2D.getNamedModels()
    models = {m: SimModel2D.Model2D(m, loadSmallerModel=True) for m in model_names}
    print('Models loaded')

# Local storage in python for each connection
StateLogsPerSID = {}
EnvironmentPerSID = {}

# Websockets
sio = socketio.Server()
app = socketio.WSGIApp(sio, static_files = static_files)

@sio.event
def connect(sid, en):
    """Stores and prints each new connection"""
    print(f"New Connection: {en['REMOTE_ADDR']} - {sid}")
    en['sid']=sid
    EnvironmentPerSID[sid] = en

def storeCSVandEnvironment(sid, path, the_time):
    """Store a csv file and environment
     param path: Folderpath to store the files in
     param the_time: time-coded filenames
    """
    os.makedirs(path, exist_ok=True) # create folder if necessary
    ## CSV
    with open(f"{path}/{the_time}.csv", 'w', newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["gripperX","gripperY","gripperRot","gripperGrasp",\
                         "targetX","targetY","box1X","box1Y","box1Rot",\
                         "box2X","box2Y","box2Rot"])
        writer.writerows(StateLogsPerSID[sid])
    del StateLogsPerSID[sid] # remove this run from local storage
    ## Environment
    with open(f"{path}/{the_time}.environ", 'w') as fp:
        json.dump(EnvironmentPerSID[sid], fp,
                  default=lambda o: '<not serializable>', indent=4)

def getFilePath():
    """Shorthand for a time-defined time path"""
    return f"{time.strftime('%Y.%m.%d_%H.%M.%S.')}"+\
                            f"{time.time():.3f}"[-3:]

@sio.on(STOP_RECORDING)
def stopRecording(sid, comment):
    """Stop recording -> store csv, environment, and comment"""
    if sid in StateLogsPerSID:
        print(f"Recording Stopped: {EnvironmentPerSID[sid]['REMOTE_ADDR']} "+\
              f"- {sid} : '{comment}'")
        path = f"{LOG_PATH}/{EnvironmentPerSID[sid]['REMOTE_ADDR']}/"
        t = getFilePath()
        storeCSVandEnvironment(sid, path, t)
        with open(f"{path}/{t}.comment", 'w') as file:
            file.write(comment)
        return "Recording Stored"

@sio.event
def disconnect(sid):
    """Handles the end of each connection"""
    if sid in StateLogsPerSID:
        print(f"Disconnect while recording: {EnvironmentPerSID[sid]['REMOTE_ADDR']}"+\
              f" - {sid}")
        storeCSVandEnvironment(sid, f"{LOG_PATH}/{EnvironmentPerSID[sid]['REMOTE_ADDR']}"+\
                               "/", getFilePath()+"_disconnect")
    else: print(f"Disconnecting: {EnvironmentPerSID[sid]['REMOTE_ADDR']} - {sid}")
    EnvironmentPerSID.pop(sid)

@sio.on(PRINT_STATELOGS)
def printStateLogs(sid):
    """Serverside: Print the internal state logs"""
    print(StateLogsPerSID)

@sio.on(CLEAR_STATELOGS)
def clearStateLogs(sid):
    """Clear the internal state logs"""
    StateLogsPerSID = {}

@sio.on(START_RECORDING)
def startRecording(sid):
    """Start recording by creating an empty state log"""
    print(f"Recording Started: {EnvironmentPerSID[sid]['REMOTE_ADDR']} - {sid}")
    StateLogsPerSID[sid] = []

@sio.on(STORE_STATE)
def storeState(sid, state):
    """Store a state log"""
    if sid in StateLogsPerSID:
        StateLogsPerSID[sid].append(state)

@sio.on(RESTARTING_SIM)
def storeRestart(sid):
    """Store a 'restart' in the data as a row of zeros"""
    if sid in StateLogsPerSID:
        if len(StateLogsPerSID[sid])>0:
            StateLogsPerSID[sid].append([0 for i in StateLogsPerSID[sid][0]])

@sio.on(UPDATE_MODELS)
def updateModels(sid):
    """Serverside: Update models"""
    loadModels()

@sio.on(MODEL_NAMES)
def sendModelNames(sid, data):
    """Send the names of all available models"""
    return model_names

@sio.on(ABOUT_REQUEST)
def sendAbout(sid, modelName):
    """Send the content of the about files for a specific model"""
    return models[modelName].getAboutFile()

@sio.on(DOF_REQUEST)
def sendDofs(sid, data):
    """Run a model on input data and send DoFs and eigenvalues to the simulation"""
    dofs, eigVals = models[data[DOF_REQUEST_MODELNAME]].updateDofs( \
        np.array(data[DOF_REQUEST_STATE]))
    return dofs.tolist(), eigVals.tolist()

@sio.on(GET_MODEL_VECTORS)
def sendModelVectors(sid, data):
    """Send a models smallerModel-calculations to the input data"""
    return models[data[DOF_REQUEST_MODELNAME]].getValue_smallerModel( \
        np.array(data[DOF_REQUEST_STATE])).tolist()

@sio.on(STORE_COMPLETE_REPLAY)
def storeCompleteReplay(sid, data):
    """Store a anonymous complete replay file for the user study without IP Addresses or environment data"""
    path = f"{LOG_PATH}/UserStudy/{sid}/"
    os.makedirs(path, exist_ok=True) # create folder if necessary
    the_time = getFilePath()
    with open(f"{path}/{the_time}.csv", 'w', newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["gripperX","gripperY","gripperRot","gripperGrasp",\
                         "targetX","targetY","box1X","box1Y","box1Rot",\
                         "box2X","box2Y","box2Rot"])
        writer.writerows(data['replay'])
    with open(f"{path}/{the_time}.comment", 'w') as file:
        json.dump(data['comment'], file, default=lambda o: '<not serializable>', indent=4)

if __name__ == '__main__':
    loadModels()
    eventlet.wsgi.server(eventlet.listen((ADDRESS, PORT)), app)
