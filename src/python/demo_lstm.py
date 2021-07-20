import argparse
import numpy as np
from toolkit.uclient import Uclient
from main_client import prepare_config
from toolkit.tools import (
    make_action, DEST_SUFFIX
)
import time
from tensorflow.keras.models import Sequential, Model, load_model, model_from_json


N = 8
ZLIM = 3
FPS = 194
TH = 0.15
UDP = False


def run_demo_lstm(my_client):
    data_queue = []
    over_threshold = False
    model = model_from_json(open('./model/lstm_model_architecture.json').read())  
    model.load_weights('./model/lstm_model_weights.h5')
    cnt = 0
    while cnt < 300:
        cnt += 1
        my_client.send_cmd(1)
        data_queue.append(list(my_client.recv_frame()[0]))
    print("start lstm")
    cnt = 0
    while True:
        # try:
        #     data = input('>> ').strip()
        #     if data and "quit".startswith(data) or data == "exit":
        #         return
        # except (EOFError, KeyboardInterrupt):
        #     return
        if cnt >= 10:
            cnt = 0
            # print(data_queue)
            if (over_threshold):
                y_predict = np.argmax(model.predict([data_queue]), axis=-1)
                print(y_predict)
                over_threshold = False
        else:
            cnt += 1
            my_client.send_cmd(1)
            del(data_queue[0])
            tmp_data = list(my_client.recv_frame()[0])
            tmp_sum = 0
            for i in tmp_data:
                tmp_sum += i
            if tmp_sum > 0.005:
                print(tmp_sum)
                over_threshold = True
            data_queue.append(tmp_data)
        # print(my_client.recv_frame()[0])



def main(args):
    config = prepare_config(args)
    with Uclient(
        config['connection']['client_address'],
        config['connection']['server_address'],
        udp=config['connection']['udp'],
        n=config['sensor']['shape']
    ) as my_client:
        print("demo lstm")
        run_demo_lstm(my_client)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--server_address', dest='server_address',
                        action=make_action('store'), help="specify server socket address")
    parser.add_argument('--client_address', dest='client_address',
                        action=make_action('store'), help="specify client socket address")
    parser.add_argument('-u', '--udp', dest='udp',
                        action=make_action('store_true'), default=UDP, help="use UDP protocol")
    parser.add_argument('-r', '--raw', dest='raw',
                        action=make_action('store_true'), default=False, help="plot raw data")
    parser.add_argument('-n', dest='n', action=make_action('store'),
                        default=[N], type=int, nargs='+', help="specify sensor shape")
    parser.add_argument('--interp', dest='interp', action=make_action('store'),
                        default=None, type=int, nargs='+', help="interpolated shape")
    parser.add_argument('--noblob', dest='noblob', action=make_action(
        'store_true'), default=False, help="do not filter out blob")
    parser.add_argument('--th', dest='threshold', action=make_action('store'),
                        default=TH, type=float, help="blob filter threshold")
    parser.add_argument('-i', '--interactive', dest='interactive',
                        action=make_action('store_true'), default=False, help="interactive mode")
    parser.add_argument('-z', '--zlim', dest='zlim', action=make_action('store'),
                        default=ZLIM, type=float, help="z-axis limit")
    parser.add_argument('-f', dest='fps', action=make_action('store'),
                        default=FPS, type=int, help="frames per second")
    parser.add_argument('-m', '--matplot', dest='matplot', action=make_action(
        'store_true'), default=False, help="use mathplotlib to plot")
    parser.add_argument('--config', dest='config', action=make_action('store'),
                        default=None, help="specify configuration file")

    parser.add_argument('--scatter', dest='scatter', action=make_action(
        'store_true'), default=False, help="show scatter plot")
    parser.add_argument('--show_value', dest='show_value',
                        action=make_action('store_true'), default=False, help="show area value")
    args = parser.parse_args()

    main(args)
