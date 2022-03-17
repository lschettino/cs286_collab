#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar  3 17:03:01 2022

@author: leticiaschettino
"""
import matplotlib.pyplot as plt 

labels = ['1', '2', '4', '8', '16', '32', '64'] 
      
a_values = [24.70, 23.44, 15.57, 7.77, 3.88, 2.33, 1.14]
b_values = [25.53, 13.61, 7.14, 3.93, 2.23, 1.37, 1.16]

plt.figure()
plt.plot(labels, a_values, label = "Parallelized code", marker="x")
plt.plot(labels, b_values, label = "Parallelized code with dynamic scheduling", marker="x")
plt.xlabel("Threads")
plt.ylabel("Wall time (s)")

plt.legend()