import pandas as pd
import os
import numpy as np

root_path = '/Users/zsz/Documents/code/Github/drcr/data/SrlgDisjoint/'
print(os.listdir(root_path))
for srlg_type in os.listdir(root_path):
    print(srlg_type)
    srlg_path = root_path + '/' + srlg_type
    if not os.path.isdir(srlg_path):
        continue
    for node_type in os.listdir(srlg_path):
        node_file = srlg_path + '/' + node_type
        if not os.path.isdir(node_file):
            continue
        for k_type in os.listdir(node_file):
            k_path = node_file + '/' + k_type
            if not os.path.isdir(k_path):
                continue
            for case_type in os.listdir(k_path):
                case_path = k_path + '/' + case_type
                if not os.path.isdir(case_path):
                    continue
                if 'tunnel_trap.csv' not in os.listdir(case_path):
                    print(case_path+'/tunnel_nontrap.csv not exist')
                    continue
                tunnel_path = case_path + '/tunnel_trap.csv'
                df = pd.read_csv(tunnel_path)
                if df.empty:
                    continue
                df = pd.read_csv(tunnel_path,header=None,skiprows=[0])
                print(tunnel_path)
                df[0] = [i for i in range(df.shape[0])]
                if df[3].any() != 0:
                    temp = df[4]
                    df[4] = df[3]
                    df[3] = temp
                if df.shape[1] > 12:
                    del df[17]
                    df.columns = ['demandID','SourceID','DestinationID','MinDelay','MaxDelay','Bandwidth','Ishotstandby','DelayDifference','DisjointType','WorkOpt','Rangetype','MinCostDelay','MinDelayDelay','ApIt','ApTime','BpIt','BpTime']
                else:
                    df.columns = ['demandID','SourceID','DestinationID','MinDelay','MaxDelay','Bandwidth','Ishotstandby','DelayDifference','DisjointType','WorkOpt','MinCostDelay','MinDelayDelay']
                df.to_csv(tunnel_path,index=None)
