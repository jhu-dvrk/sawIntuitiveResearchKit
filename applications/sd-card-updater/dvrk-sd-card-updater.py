#!/usr/bin/env python3

#
# Script used to monitor new block device (SD card) and copy new dVRK
# firmware files automatically
#
# Anton Deguet
#

import urllib.request
import os.path
import zipfile
import shutil
import pyudev
import subprocess
import time
import datetime

zip_filename = 'fpgav3-micro-sd.zip'
zip_url = 'https://github.com/jhu-cisst/mechatronics-embedded/releases/latest/download/' + zip_filename

if os.path.exists(zip_filename):
    print('Removing existing {}'.format(zip_filename))
    os.remove(zip_filename)

if os.path.exists('sd_unzipped'):
    print('Removing unzipped files')
    shutil.rmtree('sd_unzipped')

print('Downloading {}'.format(zip_url))
urllib.request.urlretrieve(zip_url, zip_filename)

print('Unzipping {}'.format(zip_filename))
with zipfile.ZipFile(zip_filename, 'r') as zip_ref:
    for zi in zip_ref.infolist():
        zip_ref.extract(zi, 'sd_unzipped')
        date_time = time.mktime(zi.date_time + (0, 0, -1))
        os.utime('sd_unzipped/' + zi.filename, (date_time, date_time))

print('Ready to write to SD card(s). Please insert you first card now.')

context = pyudev.Context()
monitor = pyudev.Monitor.from_netlink(context)
monitor.filter_by(subsystem='block')

now = datetime.datetime.now()
date_time = now.strftime("%Y-%m-%d-%H.%M.%S")

count = 0

for device in iter(monitor.poll, None):
    if device.action == 'add':
        # find the device and path
        dir(device)
        print('> udev device: {}'.format(device))
        slash_dev = device.device_node
        print('> /dev device: {}'.format(slash_dev))
        time.sleep(1.0) # 1 second to let the OS mount the device
        mount_path = subprocess.run(['findmnt', '-n', '-o', 'TARGET', slash_dev],
                                    stdout=subprocess.PIPE).stdout.decode('utf-8').strip()
        print('Mount point: {}'.format(mount_path))
        if not mount_path:
            print('Could not find mount point. Ignored.')
            continue

        existing_files = os.listdir(mount_path)
        if not (os.path.exists(mount_path + '/BOOT.bin') or len(existing_files) == 0):
            print('The drive is not empty and does not appear to be a dVRK card. Aborting. If you want to write to this drive, please empty it first.')
        else:
            # create backup dirctory if needed
            if existing_files:
                # create back dir
                backup_path = mount_path + '/backups'
                if not os.path.exists(backup_path):
                    os.mkdir(backup_path)
                backup_path = backup_path + '/' + date_time
                if not os.path.exists(backup_path):
                    os.mkdir(backup_path)
                # move files
                for f in existing_files:
                    if f != 'backups':
                        full_file = mount_path + '/' + f
                        print('Move {} to backup directory {}'.format(full_file, backup_path))
                        shutil.move(full_file, backup_path)

            # copy files
            files = os.listdir('sd_unzipped')
            for f in files:
                print('Copy {} to {}'.format(f, mount_path))
                # does not work with directories
                shutil.copy2('sd_unzipped/' + f, mount_path)

            # leave a timestamp on card
            subprocess.run(['touch', mount_path + '/created-' + date_time],
                        stdout=subprocess.PIPE)

        # and finally, umount
        umount_result = subprocess.run(['umount', mount_path],
                                       stdout=subprocess.PIPE).stdout.decode('utf-8').strip()
        print('> umount {}'.format(umount_result))

        count += 1
        print('Cart {} ready. Please insert the next card or hit ctrl+c to quit'.format(count))
