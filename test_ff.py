from time import perf_counter, sleep
import serial
import numpy as np

accs = np.array([-0.5060328,-0.54788073,-0.39142844,-0.11844139,0.19845553,0.50571099,0.76119553,0.91722007,0.91696623,0.73197078,0.4190129,0.10200847,-0.15832606,-0.33439795,-0.43610383,-0.47941837,-0.48030913,-0.45237115,-0.4063483,-0.35042212,-0.29064587,-0.23133945,-0.17542476,-0.12471647,-0.08017934,-0.0421537,-0.01054596,0.01501829,0.03507769,0.05025104,0.06117992,0.06849144,0.0727757,0.07457319,0.07436895,0.07259099,0.06961146,0.0657495,0.06127509,0.05641349,0.05134977,0.0462335,0.0411832,0.0362906,0.03162452,0.02723449,0.02315389,0.01940282,0.01599055,0.01291771,0.01017805,0.00776008,0.00564837,0.00382464,0.00226872,0.00095928,-0.00012554,-0.00100759,-0.00170834,-0.00224856,-0.00264807,-0.00292557,-0.0030985,-0.00318297,-0.00319374,-0.00314415,-0.00304621,-0.00291058,-0.00274665,-0.00256261,-0.00236549,-0.00216129,-0.00195503,-0.00175085,-0.00155209,-0.00136139,-0.00118074,-0.00101161,-0.00085497,-0.00071139,-0.00058111,-0.00046406,-0.00035997])
accs = np.array([1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,0.46602933,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,0.33437835,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,-1.,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-0.46713855,0.64157888,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,0.32550641,-0.55379494,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-0.90114885,-0.33220084,0.32596167,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,0.52801507,0.23832044,0.03424798,-0.45127703,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-0.62962001,-0.59096561,-0.90763845,-1.00000001,-0.38131838,0.78461802,1.00000001,1.00000001,1.00000001,1.00000001,0.45278296,-0.38946804,-0.49454048,0.05159708,0.7384276,1.00000001,0.65966571,-0.35798809,-1.00000001,-1.00000001,-1.00000001,-1.00000001,-0.23456789,0.51554851,0.69622386,0.38153257,-0.17404821,-0.71008611,-1.00000001,-1.00000001,-0.93090899,-0.35049948,0.44703982,0.98665571,1.00000001,0.99375499,0.95844769,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,1.00000001,0.89281776,0.70523804,0.48526735,0.26425807,0.06128182,-0.11337618,-0.25549846,-0.36486641,-0.44372375,-0.49566791,-0.52487837,-0.53560372,-0.53184216,-0.5171623,-0.49462213,-0.46675401,-0.43559098,-0.40271711,-0.36932898,-0.33630003,-0.30424232,-0.27356224,-0.24450897,-0.21721503,-0.19172917,-0.16804226,-0.14610711,-0.12585325,-0.10719755,-0.09005154])
print([np.format_float_positional(i, 2) for i in np.cumsum(accs)*0.05*1000])
curvel = 0
with serial.Serial('COM15', 250000) as ser:
    def send(acc):
        # ser.write(bytes(acc, 'utf8'))
        print(acc[:-1])
    # def send(acc): print(acc)
    # sleep(5)
    send("G91\r\n")
    print('here')
    sleep(1)
    send("G0 Z100 F6000\r\n")
    print('here again')
    sleep(5)
    dt = 0.05
    for a in accs:
        start = perf_counter()
        curvel += 1000*a*dt
        # print(curvel)
        # send(f'G0 X{1000*curvel*dt} F{1000*curvel*60}\r\n')
        send(f'G0 Z{np.format_float_positional(curvel*dt, 5)} F{np.format_float_positional(60*curvel, 5)}\r\n')

        # print(ser.read_until('\n'))
        sleep(dt/2)