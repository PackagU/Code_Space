"""Rotate-drive-rotate pose controller on odometry feedback (shared by the simulation and the Jetson tests).

Commands go to the caller's `send(v, w)`; on the robot that is /cmd_vel into nav_safety_gate (never straight to
the motors). Every cycle, before sending, `check(v, w)` (motion safety for that exact command: front, rear or
rotation clearance, stale data, e-stop) may return a stop reason: the controller then sends zero and returns that
reason at once — no further motion of any kind. `pose()` returning None (stale odometry) also stops.
The wheels follow the command late and acceleration-limited (safety gate, bridge, max_wheel_accel_rpm_s), so the base
coasts on after a zero command: the arrival is judged only once the base is at rest (`settle`). Coasted past (or
short) along the line is crept back straight, without turning (bounded). A lateral miss of more than `reaim_tol`
re-aims and drives again (bounded; smaller misses are not worth an in-place turn in a tight cabin); after the final
turn the Euclidean error must be within `arrive_tol` (in-place turns shift the position by wheel slip), otherwise
POSITION_ERROR.
While driving, a heading correction whose sweep is blocked is dropped (straight on, still checked) instead of
stopping; an in-place turn blocked one way is tried the long way round (once per phase). After stopping, a final
heading off by more than yaw_tol (coasting) is touched up slowly, at most twice. `reverse='auto'` backs up to a target that lies behind (more than 90 deg off the heading) instead of turning around:
less rotation, and the short rear (not the long fork/pallet front) leads.
"""
import math

from .geometry import wrap


def drive_to(pose, send, sleep, now, target, *, max_v=.08, max_w=.30, tol=.015, yaw_tol=.02, final_yaw=True,
             reverse=False, timeout=150.0, check=None, period=.1, retries=3, arrive_tol=.06, reaim_tol=.03,
             yaw_arrive_tol=.05, settle_s=2.0):
    tx, ty, tyaw = target
    if reverse == 'auto':
        current = pose()
        reverse = (current is not None and math.hypot(tx-current[0], ty-current[1]) > tol
                   and abs(wrap(math.atan2(ty-current[1], tx-current[0])-current[2])) > math.pi/2)
    start, phase, status, tries, creeps, rested = now(), 'face', 'REACHED', 0, 0, False
    flip = {'face': 0.0, 'turn': 0.0}           # direction (+-1) of a long-way turn after a blocked short one

    def settle():
        """Zero until the base is at rest (two still samples in a row, at most settle_s)."""
        last, still = pose(), 0
        for i in range(max(3, int(round(settle_s/period)))):
            send(0.0, 0.0)
            sleep(period)
            current = pose()
            if current is None or last is None:
                still = 2                        # stale: nothing to judge, just the stop
            else:
                moved = (math.hypot(current[0]-last[0], current[1]-last[1]) > .0005
                         or abs(wrap(current[2]-last[2])) > .001)
                still = 0 if moved else still+1
            if still >= 2 and i >= 2:
                return
            last = current

    def pulse(v, w, duration):
        """Open-loop move of exactly v*duration / w*duration from rest, then settle: a late, acceleration-limited
        base shifts the pulse in time but drives all of it (a closed loop stopping at the tolerance overshoots by
        speed x delay). Every command is checked; returns the stop reason or None."""
        steps, rest = divmod(duration, period)
        for scale in [1.0]*int(steps)+([rest/period] if rest > 1e-6 else []):
            reason = check(v*scale, w*scale) if check is not None else None
            if reason:
                settle()
                return reason
            send(v*scale, w*scale)
            sleep(period)
        settle()
        return None
    while True:                                  # main motion
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
            if along <= tol or rested:
                if not rested:                    # judge where the base stops, not where it was told to
                    settle()
                    rested = True
                    continue
                lateral = abs(dy*math.cos(yaw)-dx*math.sin(yaw))
                if lateral > max(tol, reaim_tol) and tries < retries:   # lateral miss: aim again
                    tries += 1
                    phase, rested = 'face', False
                    continue
                if abs(along) > tol and creeps < retries:    # coasted past (or short): straight, no turn
                    creeps += 1
                    creep_v = min(max_v, .02)
                    reason = pulse(math.copysign(creep_v, along)*(-1 if reverse else 1), 0.0,
                                   abs(along)/creep_v)
                    if reason:
                        status = reason
                        break
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
    settle()
    for _ in range(2):                           # at rest: up to two slow final-heading touch-ups
        current = pose()
        if (status != 'REACHED' or not final_yaw or current is None
                or abs(wrap(tyaw-current[2])) <= yaw_tol or now()-start > timeout):
            break
        err = wrap(tyaw-current[2])              # turn the rest open loop (a closed loop would overshoot again)
        touch_w = min(max_w, .05)
        status = pulse(0.0, math.copysign(touch_w, err), abs(err)/touch_w) or status
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
