#!/usr/bin/env python3

#
# Script used to disconnect and reconnect the SUJ Si Arduino BLE
#

import os
import re
import json
import subprocess
import time
import argparse
from argparse import Namespace

def load_json_dvrk(file_path:str)->dict:
    '''
    Load json files from dVRK repository
    :param file_path: json file path
    :return: a dictionary with loaded json file content
    '''
    with open(file_path) as f:
        data = f.read()
        data = re.sub("//.*?\n", "", data)
        data = re.sub("/\\*.*?\\*/", "", data)
        obj = data[data.find('{'): data.rfind('}') + 1]
        jsonObj = json.loads(obj)
    return jsonObj


def suj_bluetooth_reset(args: Namespace)->None:
    '''
    Reset the dVRK SUJ bluetooth connections
    '''
    suj_file_path = args.file_path
    data = load_json_dvrk(suj_file_path)

    address_list = []
    address_list.append(data['base-arduino-mac'])

    num_arms = len(data['arms'])

    for i_arm in range(num_arms):
        address_list.append(data['arms'][i_arm]['arduino-mac'])

    for addr in address_list:
        print(f"Disconnecting and reconnecting {addr}, timeout is 30 seconds")
        res_disconnect = subprocess.run(['bluetoothctl', 'disconnect', addr],
                                         capture_output = True, text = True,
                                         timeout = 30)
        print(res_disconnect.stdout)
        time.sleep(1.0)
        res_connect = subprocess.run(['bluetoothctl', 'connect', addr],
                                      capture_output = True, text = True,
                                      timeout = 30)
        print(res_connect.stdout)



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file_path', required = True,
                        help = 'path to dvrk SUJ Si JSON config file')

    args = parser.parse_args()
    suj_bluetooth_reset(args)

    print('SUJ Bluetooth reset complete!')
