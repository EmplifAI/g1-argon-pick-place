"""Validate a single-use checkpoint of an already stopped left-arm hold."""
import math

def validate_checkpoint(checkpoint, state, boot_id, now):
    if checkpoint.get('boot_id') != boot_id:
        raise ValueError('Holding checkpoint belongs to another boot')
    if now - checkpoint.get('saved_robot_monotonic_s', -math.inf) > 10:
        raise ValueError('Holding checkpoint expired')
    if not checkpoint.get('mechanically_supported_confirmed'):
        raise ValueError('Mechanical support is unconfirmed')
    if checkpoint.get('hand_write_enabled'):
        raise ValueError('Hand write must be disabled in saved checkpoint')
    q=checkpoint.get('target_q');h=checkpoint.get('commanded_hand_q')
    if not isinstance(q,list) or len(q)!=7 or not all(math.isfinite(v) for v in q):
        raise ValueError('Invalid left holding pose')
    if not isinstance(h,list) or len(h)!=6 or not all(math.isfinite(v) and 0<=v<=1 for v in h):
        raise ValueError('Invalid saved grip')
    if state is None or state.mode_machine!=5:
        raise ValueError('Missing compatible fresh hardware state')
    for i,m in enumerate(state.motor_state[:29]):
        if i not in range(15,22) and m.mode != 0:
            raise ValueError('Unexpected hardware mode/fault during transfer')
        if i in range(15,22) and (m.motorstate or max(m.temperature)>=88):
            raise ValueError('Arm has motor fault or overtemperature')
    generation=checkpoint.get('generation')
    if type(generation) is not int or generation<0:raise ValueError('Invalid generation')
    return q,h,generation+1
