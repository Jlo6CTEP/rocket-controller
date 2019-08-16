import struct
import matplotlib.pyplot as plt
import os

packer = struct.Struct("fffffffffffff")

FILE = "DATALOG.DAT"
PATH = "/Volumes/SD"

def temperature_of_altitude():
    f = open(os.path.join(PATH, FILE), 'rb')
    plt.figure("Temperature (Altitude)")
    amount_of_round = 1 #кол-во записей, из которых нужно взять среднее
    x, y = [], []
    while 1:
        data = []
        try:
            for i in range(amount_of_round):
                data.append(packer.unpack(f.read(packer.size)))
        except:
            break
        summ_x, summ_y, = 0, 0
        for i in range(amount_of_round):
            summ_x += data[i][9]
            summ_y += data[i][11]
        x.append(summ_x / amount_of_round)
        y.append(summ_y / amount_of_round)
    plt.xlabel('Altitude, m')
    plt.ylabel('Temperature, ℃')
    plt.scatter(y, x)
    plt.show()
    f.close()


def accel_of_time():
    f = open(os.path.join(PATH, FILE), 'rb')
    plt.figure("Acceleration (Time)")
    amount_of_round = 1 #кол-во записей, из которых нужно взять среднее
    x, x1, x2, y = [], [], [], []
    time = 0
    while 1:
        data = []
        try:
            for i in range(amount_of_round):
                data.append(packer.unpack(f.read(packer.size)))
        except:
            break
        summ_x, summ_y, summ_z = 0, 0, 0
        for i in range(amount_of_round):
            summ_x += data[i][0]
            summ_y += data[i][1]
            summ_z += data[i][2]
        x.append(summ_x / amount_of_round)
        x1.append(summ_y / amount_of_round)
        x2.append(summ_z / amount_of_round)
        y.append(time / 1000)
        for i in range(amount_of_round):
            time += data[i][12]
    plt.scatter(y, x)
    plt.xlabel('Time, s')
    plt.ylabel('Acceleration, m/s^2')
    plt.scatter(y, x1, color='#660099')
    plt.scatter(y, x2, color='#660000')
    plt.show()
    f.close()


def pressure_of_time():
    f = open(os.path.join(PATH, FILE), 'rb')
    plt.figure("Pressure (Time)")
    amount_of_round = 1 #кол-во записей, из которых нужно взять среднее
    x, y = [], []
    time = 0
    while 1:
        data = []
        try:
            for i in range(amount_of_round):
                data.append(packer.unpack(f.read(packer.size)))
        except:
            break
        summ = 0
        for i in range(amount_of_round):
            summ += data[i][10]
        x.append(summ / amount_of_round)
        y.append(time / 1000)
        for i in range(amount_of_round):
            time += data[i][12]
    plt.scatter(y, x)
    plt.xlabel('Time, s')
    plt.ylabel('Pressure, pa')
    plt.show()
    f.close()


def temperature_of_time():
    f = open(os.path.join(PATH, FILE), 'rb')
    plt.figure("Temperature (Time)")
    amount_of_round = 1 #кол-во записей, из которых нужно взять среднее
    x, y = [], []
    time = 0
    while 1:
        data = []
        try:
            for i in range(amount_of_round):
                data.append(packer.unpack(f.read(packer.size)))
        except:
            break
        summ = 0
        for i in range(amount_of_round):
            summ += data[i][9]
        x.append(summ / amount_of_round)
        y.append(time / 1000)
        for i in range(amount_of_round):
            time += data[i][12]
    plt.scatter(y, x)
    plt.xlabel('Time, s')
    plt.ylabel('Temperature, ℃')
    plt.show()
    f.close()


pressure_of_time()
temperature_of_altitude()
accel_of_time()
temperature_of_time()
