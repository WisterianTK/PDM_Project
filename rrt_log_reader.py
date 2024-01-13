import numpy as np
import json

def load_files():
    with open('rrt_log.json', "r") as file:
        rrt_log = json.load(file)
    with open('rrtu_log.json', "r") as file:
        rrtu_log = json.load(file)
    return rrt_log, rrtu_log

rrt_log, rrtu_log = load_files()

def avg(log, key):
    return sum(d[key] for d in log) / len(log)

def gen_averages(log):
    result = dict()
    for d in log:
        del d['type']
        del d['seed']
    for key in log[0].keys():
        result[key] = avg(log, key)
    return result
print('RRT-u averages:')
print(gen_averages(rrtu_log))
print('RRT averages:')
print(gen_averages(rrt_log))
