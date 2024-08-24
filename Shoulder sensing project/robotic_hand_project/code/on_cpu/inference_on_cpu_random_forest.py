import time
import serial
import pickle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons
from sklearn.ensemble import RandomForestClassifier
import threading
import multiprocessing

MAX_RECONNECT_TRIES = 5
READY = 0xAA
FREQ = 80 # Hz
COLLECT_MODE = 0 # 0 = write to file, 1 = classifier inference
OUTPUT_FILE_NAME = f"data_measurements/rest_{time.strftime('%d_%m_%Y_%H_%M_%S')}.csv"
DISPLAY_AXIS = 0
CONFIDENCE_TH = 0.6
DATA_AXES = ['Acc X', 'Acc Y', 'Acc Z', 'Gyro X', 'Gyro Y', 'Gyro Z']

def connect_to_esp():
    threading.Lock()
    esp = None
    for try_ in range(MAX_RECONNECT_TRIES):
        try:
            esp = serial.Serial(port='COM3', baudrate=115200, timeout=.1)
            if esp is not None:
                break
        except serial.SerialException as e:
            print(f"Error connecting to ESP, try {try_ + 1} of {MAX_RECONNECT_TRIES}: {e}")
    if esp is None:
        print(f"Could not connect to ESP :(")
        exit(1)
    return esp

def esp_handshake(esp: serial.Serial):
    # clear buffers etc
    esp.close()
    esp.open()
    esp.read_all()
    esp.flush()

def fetch_data(esp: serial.Serial):
    '''reads data from arduino in serial conn and put it in FIFO queue'''
    while True:
        try:
            esp.write(bytes((READY,)))
            msg = esp.read_until(b'@').decode('utf-8').strip().replace('@','').replace('#','')
            t_stamp, accelx, accely, accelz, gyrox, gyroy, gyroz, _ = msg.split(',')
            sample = (float(accelx), float(accely), float(accelz), float(gyrox), float(gyroy), float(gyroz))
            return int(t_stamp), sample
        except Exception as e:
            esp.flush()

def print_buffer(child_conn: multiprocessing.connection):
    buffer = np.zeros((1,600))
    fig, ax = plt.subplot_mosaic([['main', 'axes']],
                                  width_ratios=[4,1],
                                  layout='constrained')
    p,  = ax['main'].plot(buffer[0,DISPLAY_AXIS::6])
    ax['axes'].set_facecolor('lightgoldenrodyellow')
    axes_radio = RadioButtons(ax['axes'], DATA_AXES, active=0)
    def axesfunc(label):
        global DISPLAY_AXIS
        DISPLAY_AXIS = DATA_AXES.index(label)
    axes_radio.on_clicked(axesfunc)
    plt.show(block=False)
    plt.pause(1)
    while True:
        (buffer, delta_t) = child_conn.recv()
        p.set_ydata(buffer[0,DISPLAY_AXIS::6])
        ax['main'].set_ylim(np.min(p.get_ydata()),
                    np.max(p.get_ydata()))
        ax['main'].set_title(f'Live Buffer Axis {DATA_AXES[DISPLAY_AXIS]} rate: {int(1e6/delta_t)} Hz')
        fig.canvas.draw_idle()
        plt.pause(0.00001)

def update_buffer(buffer: np.ndarray, sample: tuple):
    return np.hstack([buffer[0,6:].reshape((1,-1)), np.array(sample).reshape((1,-1))])

def main():
    if COLLECT_MODE:
        outfile = open(OUTPUT_FILE_NAME, "wt")
        outfile.write("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ\n")
    
    idx_to_label = {
        0: 'rest',
        1: 'up',
        # 2: 'down',
        2: 'forward',
        # 4: 'back'
    }

    #connect to esp to fetch measurements
    esp = connect_to_esp()
    time_p = 0
    if not COLLECT_MODE:
        with open("code/ML/saved_RF_calssifier_2axis.pickle", 'rb') as infile:
            model: RandomForestClassifier = pickle.load(infile)
        
    buffer = np.zeros((1,600))
    now_p = 0
    # parent_conn, child_conn = multiprocessing.Pipe()
    # child_p = multiprocessing.Process(target=print_buffer, args=[child_conn], daemon=True)
    # child_p.start()
    # cnt = 0
    while True:
        now = time.perf_counter_ns()
        if now - now_p > 1e9/FREQ:
            time_, sample = fetch_data(esp)
            # print(sample)
            # print(f"elapsed: {time_ - time_p}")
            delta_t = time_ - time_p
            # print(now - now_p)
            time_p = time_
            now_p = now
            if COLLECT_MODE:
                outfile.write(f"{time_},{','.join([str(i) for i in sample])}\n")
            else:
                buffer = update_buffer(buffer, sample)
                # if cnt == 0:
                #     parent_conn.send((buffer, delta_t))
                predict = model.predict_proba(buffer)
                predicted_label = idx_to_label[np.argmax(predict)]
                if predicted_label != 'rest' and np.max(predict) > CONFIDENCE_TH:
                    print(predicted_label, predict)
            # cnt = (cnt+1)%40

if __name__ == '__main__':
    main()
