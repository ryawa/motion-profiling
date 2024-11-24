import matplotlib.pyplot as plt

fig, ax = plt.subplots(2, 2)
idx = 0

with open("data.txt") as data:
    for line in data:
        line = line.rstrip()
        if line == "":
            continue
        if line.startswith("TIME: "):
            continue
        values = list(map(float, line.split(" ")))
        if len(values) == 4:
            continue
        ax[idx % 2, idx // 2].plot(values)
        idx += 1

plt.show()
