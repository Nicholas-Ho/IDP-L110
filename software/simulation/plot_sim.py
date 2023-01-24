import matplotlib.pyplot as plt
import pandas as pd
import subprocess
import argparse

parser = argparse.ArgumentParser()

# Initial values
parser.add_argument("x")
parser.add_argument("y")
parser.add_argument("theta")

args = parser.parse_args()

# Run the C++ code
subprocess.check_call([r'controlTest.exe', args.x, args.y, args.theta])

# Read output and plot
line_width = 0.019

sim_df = pd.read_csv('output.csv', header=None, names=["X", "Y"])

plt.title('Line Following Algorithm Modeling')

# Plot path of centroid
plt.plot(sim_df["X"], sim_df["Y"])

# Plot line to follow
plt.axhline(y=line_width/2, color='r', linestyle='-')
plt.axhline(y=-line_width/2, color='r', linestyle='-')

plt.show()