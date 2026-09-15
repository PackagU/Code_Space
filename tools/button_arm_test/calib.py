"""Screen pixel -> arm press pose, from a few taught points. No kinematics model.

Camera and arm are fixed on the robot and the board is at a similar distance, so over
the small board area the press PWM changes almost linearly with the pixel position:
  1 point   -> only that button
  2 points  -> interpolate along the line between them
  3+ points -> least-squares affine map, used only inside the taught area
"""
import cv2
import numpy as np

IDS = ('000', '001', '002', '003')


def fit(points):
    if not points:
        return None
    px = np.array([p['px'] for p in points], np.float64)
    pwm = np.array([[p['press'][i] for i in IDS] for p in points], np.float64)
    radius = float(np.mean([p['radius'] for p in points]))
    model = {'n': len(points), 'px': px, 'pwm': pwm, 'radius': radius}
    design = np.hstack([px, np.ones((len(px), 1))])
    if len(points) >= 3 and np.linalg.matrix_rank(design - design.mean(axis=0), tol=10) == 2:
        model['kind'] = 'affine'
        model['coef'] = np.linalg.lstsq(design, pwm, rcond=None)[0]
        model['hull'] = cv2.convexHull(px.astype(np.float32))
    elif len(points) >= 2:
        model['kind'] = 'line'
        origin = px.mean(axis=0)
        direction = np.linalg.svd(px - origin)[2][0]
        t = (px - origin) @ direction
        model.update(origin=origin, direction=direction, t_range=(t.min(), t.max()),
                     line_coef=np.polyfit(t, pwm, 1))  # (2, 4): slope, intercept per joint
    else:
        model['kind'] = 'single'
    return model


def predict(model, u, v, radius):
    """-> ({id: pwm}, None) or (None, reason)."""
    if model is None:
        return None, '보정점 없음'
    if abs(radius - model['radius']) > .25*model['radius']:
        return None, '버튼 크기가 보정 때와 다름(거리 변화 의심)'
    margin = max(model['radius'], 25)
    point = np.array([u, v], np.float64)
    if model['kind'] == 'single':
        if np.hypot(*(point - model['px'][0])) > margin:
            return None, '보정한 버튼 위치가 아님'
        values = model['pwm'][0]
    elif model['kind'] == 'line':
        offset = point - model['origin']
        t = float(offset @ model['direction'])
        side = abs(float(offset @ np.array([-model['direction'][1], model['direction'][0]])))
        lo, hi = model['t_range']
        if side > margin or not lo - margin <= t <= hi + margin:
            return None, '보정선에서 벗어남(보정점 3개 이상 필요)'
        values = model['line_coef'][0]*t + model['line_coef'][1]
    else:
        if cv2.pointPolygonTest(model['hull'], (float(u), float(v)), True) < -margin:
            return None, '보정 영역 밖'
        values = np.array([u, v, 1.]) @ model['coef']
    return {i: int(round(x)) for i, x in zip(IDS, values)}, None
