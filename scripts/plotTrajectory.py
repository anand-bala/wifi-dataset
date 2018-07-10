#!/usr/bin/env python3
'''Plotting Trajectory

Usage:

'''
import os
import argparse

import csv
import codecs
import urllib.request

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

def load_remote_csv(url=None):
    csvfile = None
    try:
        urlstream = urllib.request.urlopen(url)
        csvfile = csv.reader(codecs.iterdecode(urlstream, 'utf-8'))
    except Exception:
        urlstream = open(url)
        csvfile = csv.reader(urlstream)
    data_x = []
    data_y = []
    for i, row in enumerate(csvfile):
        if i == 0:
            continue
        data_x.append(float(row[3]))
        data_y.append(float(row[4]))
    return data_x, data_y

def main():
    parser = argparse.ArgumentParser(description='Plot')
    parser.add_argument('csvfile', help='input csv trajectory')
    parser.add_argument('--output', help='output file for plot', default='')
    args = parser.parse_args()
    
    data_url = args.csvfile
    print('Input file:', data_url)
    print('Output file:', args.output)
    data_x, data_y = load_remote_csv(data_url)

    plt.plot(data_x, data_y)
    plt.axis('equal')
    if args.output != '':
        plt.savefig(args.output)
    plt.show()

if __name__ == '__main__':
    main()

