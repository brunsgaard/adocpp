import sys
import numpy as np
from numpy.random import randn
import pandas as pd
from scipy import stats
import matplotlib as mpl
import matplotlib.pyplot as plt
import seaborn as sns

fname = sys.argv[1]
stretch_func = lambda r, a: a/r if r != 0 and a != 0 else 1

with open(fname) as f:
    lines = f.readlines()

data = [list(map(int, l.split())) for l in lines]

stretchs = [stretch_func(*args) for args in data]


