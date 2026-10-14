"""Rotate-drive-rotate pose controller on odometry feedback (shared by the simulation and the Jetson tests).

Commands go to the caller's `send(v, w)`; on the robot that is /cmd_vel into nav_safety_gate (never straight to
the motors). Every cycle, before sending, `check(v, w)` (motion safety for that exact command: front, rear or
rotation clearance, stale data, e-stop) may return a stop reason: the controller then sends zero and returns that
reason at once — no further motion of any kind. `pose()` returning None (stale odometry) also stops.
A lateral miss of more than `reaim_tol` re-aims and drives again (bounded; smaller misses are not worth an in-place
turn in a tight cabin); after the final turn the Euclidean error must be within `arrive_tol` (in-place turns shift
the position by wheel slip), otherwise POSITION_ERROR.
While driving, a heading correction whose sweep is blocked is dropped (straight on, still checked) instead of
stopping; an in-place turn blocked one way is tried the long way round (once per phase). After stopping, a final
heading off by more than yaw_tol (coasting) is touched up slowly, at most twice. `reverse='auto'` backs up to a target that lies behind (more than 90 deg off the heading) instead of turning around:
less rotation, and the short rear (not the long fork/pallet front) leads.
"""
import math

from .geometry import wrap


def drive_to(pose, send, sleep, now, target, *, max_v=.08, max_w=.30, tol=.015, yaw_tol=.02, final_yaw=True,
             reverse=False, timeout=150.0, check=None, period=.1, retries=3, arrive_tol=.06, reaim_tol=.03,
             yaw_arrive_tol=.05):
    tx, ty, tyaw = target
    if reverse == 'auto':
        current = pose()
        reverse = (current is not None and math.hypot(tx-current[0], ty-current[1]) > tol
                   and abs(wrap(math.atan2(ty-current[1], tx-current[0])-current[2])) > math.pi/2)
    start, phase, status, tries = now(), 'face', 'REACHED', 0
    flip = {'face': 0.0, 'turn': 0.0}           # direction (+-1) of a long-way turn after a blocked short one
    settles = 0
    while True:                                  # main motion, then up to two slow final-heading touch-ups
        while True:
            if now()-start > timeout:
                status = 'TIMEOUT'
                break
            current = pose()
            if current is None:
                status = 'STALE_POSE'
                break
            x, y, yaw = current
            dx, dy = tx-x, ty-y
            dist = math.hypot(dx, dy)
            heading = math.atan2(dy, dx)+(math.pi if reverse else 0.0)
            v = w = 0.0
            if phase == 'face':
                if dist <= max(tol, reaim_tol):           # not worth an in-place turn to creep a few cm
                    phase = 'turn'
                    continue
                err = wrap(heading-yaw)
                if abs(err) < .02:
                    phase = 'drive'
                    continue
                w = math.copysign(min(max_w, max(.05, 1.2*abs(err))), err)
                if flip['face']:
                    w = flip['face']*max_w
            elif phase == 'drive':
                along = (dx*math.cos(yaw)+dy*math.sin(yaw))*(-1 if reverse else 1)
                if along <= tol:
                    if dist > max(tol, reaim_tol) and tries < retries:      # lateral miss: aim again
                        tries += 1
                        phase = 'face'
                        continue
                    phase = 'turn'
                    continue
                err = wrap(heading-yaw)
                v = min(max_v, .5*along+.02)*(-1 if reverse else 1)
                w = max(-max_w, min(max_w, 1.5*err)) if dist > .05 else 0.0
            else:
                if not final_yaw:
                    break
                err = wrap(tyaw-yaw)
                if abs(err) < yaw_tol:
                    break
                w = math.copysign(min(max_w, max(.05, 1.2*abs(err))), err)
                if flip['turn']:
                    w = flip['turn']*max_w
            reason = check(v, w) if check is not None else None
            if reason == 'SWEEP_STOP' and phase in flip and not flip[phase] and abs(v) <= 1e-3:
                # an in-place turn blocked one way may be free the other way round
                w = -math.copysign(max_w, w)
                flip[phase] = math.copysign(1.0, w)
                reason = check(v, w)
            if reason == 'SWEEP_STOP' and phase == 'drive' and abs(v) > 1e-3:
                # the heading correction is optional while driving: go straight on if that alone is clear
                w = 0.0
                reason = check(v, w)
            if reason:
                status = reason
                break
            send(v, w)
            sleep(period)
        for _ in range(3):
            send(0.0, 0.0)
            sleep(period/2)
        current = pose()
        if (status != 'REACHED' or not final_yaw or current is None or settles >= 2
                or abs(wrap(tyaw-current[2])) <= yaw_tol or now()-start > timeout):
            break
        settles += 1                             # coasting after the stop left the heading off: turn again
        phase, max_w = 'turn', min(max_w, .1)
    current = pose()
    if current is None:
        return {'status': status if status != 'REACHED' else 'STALE_POSE'}
    x, y, yaw = current
    err_xy = math.hypot(tx-x, ty-y)
    if status == 'REACHED' and err_xy > max(tol, arrive_tol):
        status = 'POSITION_ERROR'
    elif status == 'REACHED' and final_yaw and abs(wrap(tyaw-yaw)) > max(yaw_tol, yaw_arrive_tol):
        status = 'YAW_ERROR'                     # still off after the touch-ups
    return {'status': status, 'pos_err_m': round(err_xy, 4),
            'yaw_err_rad': round(abs(wrap(tyaw-yaw)), 4) if final_yaw else None}
