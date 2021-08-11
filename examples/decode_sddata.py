
# -*- coding: utf-8 -*-
"""
Helper to decode binary logged sensor data from crazyflie2 with uSD-Card-Deck
"""
import argparse
from zlib import crc32
import re
import struct
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import csv

write_csv = True
log_filename = "log00"

# extract null-terminated string
def _get_name(data, idx):
    endIdx = idx
    while data[endIdx] != 0:
        endIdx = endIdx + 1
    return data[idx:endIdx].decode("utf-8"), endIdx + 1

def decode(filename):
    # read file as binary
    with open(filename, 'rb') as f:
        data = f.read()

    # check magic header
    if data[0] != 0xBC:
        print("Unsupported format!")
        return

    # check CRC
    crc = crc32(data[0:-4])
    expected_crc, = struct.unpack('I', data[-4:])
    if crc != expected_crc:
        print("WARNING: CRC does not match!")

    # check version
    version, num_event_types = struct.unpack('HH', data[1:5])
    if version != 1 and version != 2:
        print("Unsupported version!", version)
        return

    result = dict()
    event_by_id = dict()

    # read header with data types
    idx = 5
    for _ in range(num_event_types):
        event_id, = struct.unpack('H', data[idx:idx+2])
        idx += 2
        event_name, idx = _get_name(data, idx)
        result[event_name] = dict()
        result[event_name]["timestamp"] = []
        num_variables, = struct.unpack('H', data[idx:idx+2])
        idx += 2
        fmtStr = "<"
        variables = []
        for _ in range(num_variables):
            var_name_and_type, idx = _get_name(data, idx)
            var_name = var_name_and_type[0:-3]
            var_type = var_name_and_type[-2]
            result[event_name][var_name] = []
            fmtStr += var_type
            variables.append(var_name)
        event_by_id[event_id] = {
            'name': event_name,
            'fmtStr': fmtStr,
            'numBytes': struct.calcsize(fmtStr),
            'variables': variables,
            }

    while idx < len(data) - 4:
        if version == 1:
            event_id, timestamp, = struct.unpack('<HI', data[idx:idx+6])
            idx += 6
        elif version == 2:
            event_id, timestamp, = struct.unpack('<HQ', data[idx:idx+10])
            timestamp = timestamp / 1000.0
            idx += 10
        event = event_by_id[event_id]
        fmtStr = event['fmtStr']
        eventData = struct.unpack(fmtStr, data[idx:idx+event['numBytes']])
        idx += event['numBytes']
        for v,d in zip(event['variables'], eventData):
            result[event['name']][v].append(d)
        result[event['name']]["timestamp"].append(timestamp)

    # remove keys that had no data
    for event_name in list(result.keys()):
        if len(result[event_name]['timestamp']) == 0:
            del result[event_name]

    # convert to numpy arrays
    for event_name in result.keys():
        for var_name in result[event_name]:
            result[event_name][var_name] = np.array(result[event_name][var_name])

    return result


if __name__ == "__main__":
    #parser = argparse.ArgumentParser()
    #parser.add_argument("filename")
    #args = parser.parse_args()
    #args.filename = "/home/guillermoga/sd_logs/log00"
    logData = decode('/home/guillermoga/sd_logs/'+log_filename)

     #only focus on regular logging
    logData = logData['fixedFrequency']

    # set window background to white
    plt.rcParams['figure.facecolor'] = 'w'
        
    # number of columns and rows for suplot
    plotCols = 1
    plotRows = 1

    # let's see which keys exists in current data set
    keys = []
    for k, v in logData.items():
        keys.append(k)
    print(keys)


    if write_csv:
        with open("/home/guillermoga/sd_logs/" + log_filename + ".csv", mode='w') as csv_file:
            writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow([header for header in keys])

            for i in range(len(logData['timestamp'])):
                writer.writerow([logData[key][i] for key in keys])

  