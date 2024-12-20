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
# ax[0, 0].plot(vels, label="Velocity")
# ax[0, 0].set_title("Velocity")
# ax[0, 0].legend()

# ax[0, 1].plot(angular_vels, label="Angular Velocity")
# ax[0, 1].set_title("Angular Velocity")
# ax[0, 1].legend()

# ax[1, 0].plot(accels, label="Acceleration")
# ax[1, 0].set_title("Acceleration")
# ax[1, 0].legend()

# ax[1, 1].plot(angular_accels, label="Angular Acceleration")
# ax[1, 1].set_title("Angular Acceleration")
# ax[1, 1].legend()
# track_width = 12.375
track_width = 1000
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


# xoff = 9.5
xoff = 0
yoff = 0.0
# yoff = 4
x = 0
y = 0
heading = 0
dt = 0.001
d = 0
i = 0


while i < len(vels):
    # print(d)
    vel = vels[i]
    if vel == 0:
        vel = 0.1
    angular_vel = angular_vels[i]
    # angular_vel *= 0.5
    left = vel - angular_vel * track_width / 2
    right = vel + angular_vel * track_width / 2
    vel = (left + right) / 2
    angular_vel = (right - left) / track_width
    d += vel * dt
    x += vel * math.cos(heading) * dt
    y += vel * math.sin(heading) * dt

    dtheta = angular_vel * dt
    xcenter = xoff * math.cos(heading) - yoff * math.sin(heading) + x
    ycenter = xoff * math.sin(heading) + yoff * math.cos(heading) + y
    x -= xcenter
    y -= ycenter
    xnew = x * math.cos(dtheta) - y * math.sin(dtheta)
    ynew = x * math.sin(dtheta) + y * math.cos(dtheta)
    x = xnew + xcenter
    y = ynew + ycenter

    heading += dtheta
    ax.plot(x, y, "r.")
    ax.plot(xcenter, ycenter, "b.")
    i = int(d // 0.1)
print(xoff, yoff, x, y)

plt.axis("equal")
plt.show()
