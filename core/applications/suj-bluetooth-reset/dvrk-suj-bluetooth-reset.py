import os
import sys
import re
import json
import subprocess
import time
import argparse
from argparse import Namespace

dynamic_path = os.path.abspath(__file__+"/../../../../../../")
# dynamic_path = os.path.abspath(__file__+"/../../")  # Phase 2 folder
# print(dynamic_path)
sys.path.append(dynamic_path)

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

    for i in range(len(address_list)):
        res_disconnect = subprocess.run(['bluetoothctl', 'disconnect', address_list[i]], capture_output=True, text=True)
        print(res_disconnect.stdout)
        time.sleep(1.0)
        res_connect = subprocess.run(['bluetoothctl', 'connect', address_list[i]], capture_output=True, text=True)
        print(res_connect.stdout)



if __name__ == '__main__':
    dvrk_config_folder = os.path.join(dynamic_path, 'dvrk', 'dvrk_config_jhu', 'jhu-daVinci-Si')
    dvrk_suj_file = os.path.join(dvrk_config_folder, 'suj-si.json')

    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file_path', default=dvrk_suj_file,
                        help='path to dvrk SUJ config file')

    args = parser.parse_args()
    suj_bluetooth_reset(args)

    print('SUJ Bluetooth reset complete!')
