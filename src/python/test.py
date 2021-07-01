import argparse
import re
import os

def dispatch(caseNo):
    if caseNo == '01-11':
        os.system('python main_server.py')
    elif caseNo == '08-01' or caseNo == '08-02':
        os.system('python main_imu_visual.py')
    else:
        print('Unrecognized case no.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='开始测试')
    parser.add_argument('case', metavar='No', type=str, nargs=1)
    args = parser.parse_args()
    assert(len(args.case) == 1)
    if not re.search('0[1-8]-[01][0-9]', args.case[0]):
        print('Invalid case no.')
    else:
        dispatch(args.case[0])