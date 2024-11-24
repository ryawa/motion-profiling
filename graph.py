import math

import matplotlib.pyplot as plt
import numpy as np

# fig, ax = plt.subplots(2, 2)
fig, ax = plt.subplots()
idx = 0

with open("data.txt") as data:
    for line in data:
        line = line.rstrip()
        values = line.split(" ")
        match values[0]:
            case "VELS:":
                vels = list(map(float, values[1:]))
            case "ANGULARVELS:":
                angular_vels = list(map(float, values[1:]))
            case "ACCELS:":
                accels = list(map(float, values[1:]))
            case "ANGULARACCELS:":
                angular_accels = list(map(float, values[1:]))

track_width = 12.1875
# for i, (vel, angular_vel) in enumerate(zip(vels, angular_vels)):
#     ax[0, 0].plot(i, vel - angular_vel * track_width / 2, "r.")
#     ax[0, 1].plot(i, vel + angular_vel * track_width / 2, "r.")
# for i, (accel, angular_accel) in enumerate(zip(accels, angular_accels)):
#     ax[1, 0].plot(i, accel - angular_accel * track_width / 2, "b.")
#     ax[1, 1].plot(i, accel + angular_accel * track_width / 2, "b.")


# # Integrate acceleration data to get velocity
# integrated_vels = np.cumsum(accels) * 0.1  # Assuming data is sampled every 0.1 inches
# integrated_angular_vels = np.cumsum(angular_accels) * 0.1
# # Calculate dt from velocity and ds
# dt = [0.1 / vel if vel != 0 else 0 for vel in vels]

# # Integrate acceleration data to get velocity using dt
# integrated_vels_dt = np.cumsum([accel * dt_i for accel, dt_i in zip(accels, dt)])
# integrated_angular_vels_dt = np.cumsum(
#     [angular_accel * dt_i for angular_accel, dt_i in zip(angular_accels, dt)]
# )

# # Plot integrated velocities using dt
# for i, (vel, angular_vel) in enumerate(
#     zip(integrated_vels_dt, integrated_angular_vels_dt)
# ):
#     ax[1, 0].plot(i, vel - angular_vel * track_width / 2, "m.")
#     ax[1, 1].plot(i, vel + angular_vel * track_width / 2, "m.")

x = 0
y = 0
heading = math.pi / 4
dt = 0.01
d = 0
i = 0
while i < len(vels):

    print(d)
    vel = vels[i]
    if vel == 0:
        vel = 0.1
    angular_vel = angular_vels[i]
    left = vel - angular_vel * track_width / 2
    right = vel + angular_vel * track_width / 2
    d += vel * dt
    x += vel * math.cos(heading) * dt
    y += vel * math.sin(heading) * dt
    heading += angular_vel * dt
    ax.plot(x, y, "r.")
    i = int(d // 0.1)


plt.axis("equal")
plt.show()
