"""Factory rt/arm_sdk command: upper-body fields only, no hand registers."""
import math
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

def build_command(q, weight):
    if len(q)!=29 or not all(math.isfinite(v) for v in q) or not 0<=weight<=1:
        raise ValueError('Invalid upper-body SDK command')
    msg=unitree_hg_msg_dds__LowCmd_()
    msg.mode_pr=0;msg.mode_machine=5
    for i in range(12,29):
        m=msg.motor_cmd[i]
        m.q=q[i];m.dq=0;m.kp=60 if i<19 or 22<=i<26 else 30;m.kd=1.5;m.tau=0
    msg.motor_cmd[29].q=weight
    msg.crc=CRC().Crc(msg)
    return msg
