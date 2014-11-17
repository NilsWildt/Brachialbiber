#!/usr/bin/python
# coding: utf-8

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d



def main():

    #Strip Whitespace, doens't work correctyl --> First backup output ...
    with open("AAAoutput.txt", "r") as f:
        for line in f:
            cleanedLine = line.strip()
            if cleanedLine: #not empty
                print(cleanedLine)
    y = np.genfromtxt('AAAoutput.txt', delimiter = '\n') 
    print(len(y))
    x = range(1,len(y)+1)
    f = interp1d(x,y,kind='cubic')

    fig, ax = plt.subplots()
    ax.plot(x, f, label='AngleVelocity')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05),
          fancybox=True, shadow=True, ncol=5)
    ax.set_xlabel('$x$')
	ax.set_ylim([0,100])
    ax.set_ylim([0,50])
    ax.set_ylabel('$y$')
    ax.set_title('Werte des Gyros');
    fig.savefig("AngleVelocity.svg")
    plt.show(); 
    
if __name__ == "__main__":
    main()
    
    
    