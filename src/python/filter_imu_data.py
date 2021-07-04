import json
import numpy as np

FRAME_LEN = 100

def checkStatic(data):
    varValue = np.var(data, axis = 0)
    varNorm = np.linalg.norm(varValue)
    return varNorm < 0.01
if __name__ == '__main__':
    data = []
    with open('imu_calibration_data.txt') as f:
        for line in f:
            d = json.loads(line.strip())
            assert(len(d) == 3)
            data.append(d)
            if len(data) > FRAME_LEN:
                if checkStatic(data):
                    print(np.mean(data, axis = 0))
                    data = []
                else:
                    data = data[-FRAME_LEN:]
