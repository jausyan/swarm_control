import numpy as np
import matplotlib.pyplot as plt

# Posisi awal drone
drone1 = np.array([2, 0])
drone2 = np.array([5, 0])
drone3 = np.array([8, 0])

drones = np.array([drone1, drone2, drone3])

# Sudut rotasi (90 derajat)
theta = np.pi / 2

# Matriks rotasi
R = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta),  np.cos(theta)]
])

# Rotasi semua drone (terhadap drone 1 / origin)
rotated_drones = drones @ R.T

# Plot
plt.figure()
    
# Posisi awal
plt.scatter(drones[:,0], drones[:,1])
for i, (x, y) in enumerate(drones):
    plt.text(x, y, f"D{i+1} awal")

# Posisi setelah rotasi
plt.scatter(rotated_drones[:,0], rotated_drones[:,1], marker='x')
for i, (x, y) in enumerate(rotated_drones):
    plt.text(x, y, f"D{i+1} rot")

# Garis bantu
plt.plot(drones[:,0], drones[:,1], linestyle='--')
plt.plot(rotated_drones[:,0], rotated_drones[:,1], linestyle='--')

plt.axhline(0)
plt.axvline(0)

plt.title("Rotasi Formasi Drone 90 Derajat")
plt.axis('equal')
plt.grid()

plt.show()