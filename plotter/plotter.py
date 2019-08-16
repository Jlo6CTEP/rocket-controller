import os
import struct
import matplotlib.pyplot as plt

FILE = "datalog.dat"
PATH = "E:"
IS_PRINT = True

data = list(struct.iter_unpack("f"*13, open(os.path.join(PATH, FILE), "rb").read()))

if IS_PRINT:
    for x in data:
        print(("{:+15.4f}" * 13).format(*x))


plt.figure(0)
plt.plot([x[9] for x in data])
plt.ylabel("temperature")
plt.xlabel("time")

plt.figure(1)
plt.plot([x[10]/1000 for x in data])
plt.ylabel("pressure")
plt.xlabel("time")
plt.show()
