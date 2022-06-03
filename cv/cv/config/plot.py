import matplotlib.pyplot as plt
import json

data = json.load(open('depth_vals_planar.json'))

fig = plt.figure(figsize=(4,4))
ax = fig.add_subplot(111, projection='3d')
x, y, z = [], [], []

for key, value in data.items():
    key = key[1:-1]
    key = key.replace(", ", " ")
    key = key.split()
    x.append(int(key[0]))
    y.append(int(key[1]))
    z.append(float(value))

ax.scatter(x, y, z)
plt.show()