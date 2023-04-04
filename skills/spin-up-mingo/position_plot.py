import numpy as np
import matplotlib.pyplot as plt
import pandas as pd 


df = pd.read_csv('positions.csv', names=['time [ms]', 'transverseEncoderValue', 'positionY', 'radialEncoderValue', 'positionX'])
plt.figure(figsize=(15,10))

plt.subplot(121)
plt.plot(df.iloc[:,0],df.iloc[:,1], label='transverse')
plt.plot(df.iloc[:,0],df.iloc[:,3], '-.', label='radial')
plt.title("EncoderValue vs Time")
plt.xlabel('time [ms]')
plt.ylabel('EncoderValue')
plt.grid(True)
plt.legend()

plt.subplot(122)
plt.plot(df.iloc[:,0],df.iloc[:,2], label='positionY')
plt.plot(df.iloc[:,0],df.iloc[:,4], '-.', label='positionX')
plt.title('Position vs Time')
plt.xlabel('time [ms]')
plt.ylabel('Position')
plt.grid(True)
plt.legend()

plt.show()