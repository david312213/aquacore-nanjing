"""Preparation-only local timing of an unchanged MINCO spatial curve.

q(tau) stays intact.  d tau / dt = exp(L(tau)), where L is a C2 cubic
spline.  Only the preparer imports scipy; the 50 Hz sampler uses numpy and
small, fixed quadrature tables.  An unavailable/unsuccessful optimization
returns the existing reference, never a new mission admission condition.

Optional dynamics limits (planning.retiming) replace the isotropic
acceleration budget by what a multirotor actually spends: the thrust
direction f = a + k_drag v + g z.  The feed-forward tilt, total specific
thrust and tilt rate |f x f'| / |f|^2 are bounded so that PX4 keeps margin
for feedback.  They only shape the reference time; they never block flight.
"""
from bisect import bisect_right
import math

import numpy as np
from numpy.polynomial import polynomial as poly


_GAUSS, _WEIGHTS = np.polynomial.legendre.leggauss(16)


class LocalTimeMap:
    """Serializable positive time mapping; derivatives are analytic.

    Tables bracket the inverse only. Position/velocity/acceleration/jerk are
    not linearly interpolated. Short-interval quadrature and Newton inversion
    evaluate the smooth map between table entries, including at waypoints.
    """
    def __init__(self, value):
        self.value = value
        self.knots = np.asarray(value['knots'], dtype=float)
        self.coefficients = np.asarray(value['log_rate_coefficients'], dtype=float)
        self.tau = np.asarray(value['table_tau'], dtype=float)
        self.elapsed = np.asarray(value['table_elapsed'], dtype=float)
        self.duration = float(self.elapsed[-1])

    @staticmethod
    def _integral(coefficients, duration, u0, u1):
        u = u0+(_GAUSS+1.)*.5*(u1-u0)
        return float(np.dot(_WEIGHTS, np.exp(-poly.polyval(u, coefficients))) *
                     (.5*(u1-u0)*duration))

    @classmethod
    def prepare(cls, knots, coefficients):
        tau, elapsed = [float(knots[0])], [0.]
        # Equal normalized intervals keep inversion local even for short knots.
        for i, c in enumerate(coefficients):
            duration = knots[i+1]-knots[i]
            for j in range(32):
                tau.append(float(knots[i]+duration*(j+1)/32.))
                elapsed.append(elapsed[-1]+cls._integral(c, duration, j/32., (j+1)/32.))
        return cls(dict(version=1, knots=list(knots),
                        log_rate_coefficients=np.asarray(coefficients).tolist(),
                        table_tau=tau, table_elapsed=elapsed))

    def time_at_base(self, tau):
        if tau <= 0.:
            return 0.
        if tau >= self.knots[-1]:
            return self.duration
        j = max(0, bisect_right(self.tau, tau)-1)
        i = min(bisect_right(self.knots, tau)-1, len(self.coefficients)-1)
        duration = self.knots[i+1]-self.knots[i]
        u0, u1 = (self.tau[j]-self.knots[i])/duration, (tau-self.knots[i])/duration
        return float(self.elapsed[j]+self._integral(self.coefficients[i], duration, u0, u1))

    def state(self, elapsed):
        elapsed = min(self.duration, max(0., float(elapsed)))
        j = min(max(0, bisect_right(self.elapsed, elapsed)-1), len(self.elapsed)-2)
        # Every table interval lies wholly within a polynomial segment.
        i = min(j//32, len(self.coefficients)-1)
        duration = self.knots[i+1]-self.knots[i]
        c = self.coefficients[i]
        u0 = (self.tau[j]-self.knots[i])/duration
        u1 = (self.tau[j+1]-self.knots[i])/duration
        target = elapsed-self.elapsed[j]
        u = u0+(u1-u0)*target/(self.elapsed[j+1]-self.elapsed[j])
        low, high = u0, u1
        # Numerical inverse only: no flight-state checks, waits or retries.
        for _ in range(8):
            residual = self._integral(c, duration, u0, u)-target
            if abs(residual) <= 2e-14*max(1., duration):
                break
            if residual > 0.:
                high = u
            else:
                low = u
            update = u-residual*math.exp(poly.polyval(u, c))/duration
            u = update if low <= update <= high else (low+high)*.5
        log_rate = poly.polyval(u, c)
        slope = (c[1]+u*(2.*c[2]+u*3.*c[3]))/duration
        curvature = (2.*c[2]+u*6.*c[3])/duration**2
        rate = math.exp(log_rate)
        return (float(self.knots[i]+u*duration), rate,
                rate**2*slope, rate**3*(2.*slope*slope+curvature))


def _evaluate(trajectory, tau, orders=(1, 2)):
    index = np.minimum(np.searchsorted(trajectory._ends, tau, side='right'),
                       len(trajectory.durations)-1)
    duration = np.asarray(trajectory.durations)[index]
    unit = (tau-np.asarray(trajectory._starts)[index])/duration
    result = []
    for order in orders:
        rows = trajectory._derivatives[order][index]
        value = np.zeros((len(tau), 3))
        for k in range(rows.shape[-1]-1, -1, -1):
            value = value*unit[:, None]+rows[:, :, k]
        result.append(value/duration[:, None]**order)
    return result


def _cap_windows(node_limits, knots):
    windows, index = [], 0
    while index < len(node_limits):
        if node_limits[index] is None:
            index += 1
            continue
        end = index
        while end+1 < len(node_limits) and node_limits[end+1] is not None:
            end += 1
        windows.append(dict(first=index, last=end, tau0=knots[index], tau1=knots[end],
                            cap=min(node_limits[index:end+1])))
        index = end+1
    return windows


def _extrema(trajectory, coefficients, windows, max_velocity, max_acceleration):
    """Stationary roots of exp(k L) times squared polynomial derivatives.

    The positive exponential factors out of the extremum equation. Include
    both ends and isolated capped nodes, not just the optimization mesh.
    """
    peak_v, peak_a, scale = 0., 0., 1.
    gate_peaks = [0.]*len(windows)
    for i, duration in enumerate(trajectory.durations):
        log_rate = coefficients[i]
        slope = poly.polyder(log_rate)/duration
        d1 = trajectory._derivatives[1][i]/duration
        d2 = trajectory._derivatives[2][i]/duration**2
        acceleration = [poly.polyadd(d2[k], poly.polymul(d1[k], slope)) for k in range(3)]
        maxima = []
        for rows, power in ((d1, 2), (acceleration, 4)):
            squared = np.array([0.])
            for row in rows:
                squared = poly.polyadd(squared, poly.polymul(row, row))
            derivative = poly.polyadd(poly.polyder(squared),
                                     power*poly.polymul(squared, poly.polyder(log_rate)))
            roots = (poly.polyroots(derivative/np.max(np.abs(derivative)))
                     if np.any(derivative) else [])
            candidates = [0., 1.]+[r.real for r in roots if abs(r.imag) < 1e-6 and 0. < r.real < 1.]
            # Supplement near-multiple roots with a separate dense mesh.
            candidates.extend(np.linspace(0., 1., 101))
            maxima.append(max(math.exp(power*poly.polyval(u, log_rate)/2.) *
                              math.sqrt(sum(poly.polyval(u, row)**2 for row in rows))
                              for u in candidates))
        peak_v, peak_a = max(peak_v, maxima[0]), max(peak_a, maxima[1])
        for j, window in enumerate(windows):
            if window['first'] <= i < window['last']:
                gate_peaks[j] = max(gate_peaks[j], maxima[0])
            for node, u in ((i, 0.), (i+1, 1.)):
                if window['first'] <= node <= window['last']:
                    speed = math.exp(poly.polyval(u, log_rate))*float(np.linalg.norm(
                        trajectory._sample_segment(i, u, 1)))
                    gate_peaks[j] = max(gate_peaks[j], speed)
    scale = max(scale, peak_v/max_velocity, math.sqrt(peak_a/max_acceleration),
                max((peak/w['cap'] for peak, w in zip(gate_peaks, windows)), default=0.))
    return dict(speed=peak_v, acceleration=peak_a, gate_peaks=gate_peaks, scale=scale)


GRAVITY = 9.80665
_LOG_EPS = .05
DYNAMICS_DEFAULTS = dict(feedforward_tilt_deg=30., tilt_rate_dps=120., drag_per_s=.25,
                         max_thrust_g=1.3, min_thrust_g=.5, smoothing=0.)


TIMING_MODES = ('original', 'optimized', 'thrust')
TIMING_LABELS = dict(original='原配时', optimized='局部配时（速度/加速度上限）',
                     thrust='局部配时（推力方向约束）')


def timing_mode(config, default='original'):
    """navigation.timing: original | optimized | thrust (null -> default)."""
    value = (config.get('navigation') or {}).get('timing')
    value = default if value is None else value
    if value not in TIMING_MODES:
        raise ValueError('navigation.timing 须为 original、optimized 或 thrust，当前: '+str(value))
    return value


def timing_dynamics(config, mode):
    """Thrust budgets for mode 'thrust' (planning.retiming, defaults filled); else None."""
    return dynamics_limits(config.get('planning')) if mode == 'thrust' else None


def dynamics_limits(planning):
    """planning.retiming -> validated thrust-direction budgets with defaults.

    Only used when navigation.timing is 'thrust'.  Values are
    reference-shaping budgets only, not flight admission checks.
    """
    value = (planning or {}).get('retiming')
    if value is None:
        value = {}
    if not isinstance(value, dict):
        raise ValueError('planning.retiming 须为字典或 null')
    unknown = set(value)-set(DYNAMICS_DEFAULTS)
    if unknown:
        raise ValueError('planning.retiming 未知参数: '+', '.join(sorted(unknown)))
    result = dict(DYNAMICS_DEFAULTS, **{k: v for k, v in value.items() if v is not None})
    for key, number in result.items():
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
            raise ValueError('planning.retiming.%s 须为有限数值' % key)
        result[key] = float(number)
    if not 5. <= result['feedforward_tilt_deg'] <= 60.:
        raise ValueError('planning.retiming.feedforward_tilt_deg 须在 5 到 60 之间')
    if result['tilt_rate_dps'] <= 0. or result['drag_per_s'] < 0. or result['smoothing'] < 0.:
        raise ValueError('planning.retiming 角速度须为正，阻力与平滑系数不能为负')
    if not 0. < result['min_thrust_g'] < 1. < result['max_thrust_g']:
        raise ValueError('planning.retiming 须满足 0 < min_thrust_g < 1 < max_thrust_g')
    return result


def _thrust_terms(d1, d2, d3, lograte, slope, curvature, dynamics):
    """Velocity, specific thrust f and its time derivative from tau-domain data.

    With r = exp(L): v = r q1, a = r^2 (q2 + L1 q1),
    j = r^3 (q3 + 3 L1 q2 + (2 L1^2 + L2) q1), where qn / Ln are tau derivatives.
    """
    r = np.exp(lograte)[:, None]
    s1, s2 = slope[:, None], curvature[:, None]
    k = dynamics['drag_per_s']
    v = r*d1
    a = r**2*(d2+s1*d1)
    j = r**3*(d3+3.*s1*d2+(2.*s1*s1+s2)*d1)
    f = a+k*v
    f[:, 2] += GRAVITY
    return v, a, j, f, j+k*a


def _dynamics_ratios(f, fdot, dynamics):
    """Each budget as a ratio; <= 1 everywhere means the reference fits."""
    tan_tilt = math.tan(math.radians(dynamics['feedforward_tilt_deg']))
    norm2 = np.sum(f*f, axis=1)
    fz = f[:, 2]
    horizontal = np.sqrt(f[:, 0]**2+f[:, 1]**2)
    rate = np.linalg.norm(np.cross(f, fdot), axis=1)/np.maximum(norm2, 1e-12)
    return dict(
        tilt=horizontal/np.maximum(fz, 1e-9)/tan_tilt,
        thrust=np.sqrt(norm2)/(dynamics['max_thrust_g']*GRAVITY),
        lift=dynamics['min_thrust_g']*GRAVITY/np.maximum(fz, 1e-9),
        tilt_rate=rate/math.radians(dynamics['tilt_rate_dps']))


def _dense_dynamics(trajectory, coefficients, dynamics, count=201):
    """Dense tau grid of the retimed reference: f, f', ratios."""
    rows = []
    for i, duration in enumerate(trajectory.durations):
        u = np.linspace(0., 1., count)
        c = coefficients[i]
        tau = trajectory._starts[i]+u*duration
        lograte = poly.polyval(u, c)
        slope = poly.polyval(u, poly.polyder(c))/duration
        curvature = poly.polyval(u, poly.polyder(c, 2))/duration**2
        rows.append((tau, lograte, slope, curvature))
    tau, lograte, slope, curvature = (np.concatenate(x) for x in zip(*rows))
    d1, d2, d3 = _evaluate(trajectory, tau, (1, 2, 3))
    v, a, j, f, fdot = _thrust_terms(d1, d2, d3, lograte, slope, curvature, dynamics)
    return dict(tau=tau, lograte=lograte, f=f, fdot=fdot, velocity=v,
                ratios=_dynamics_ratios(f, fdot, dynamics))


def _bisect_shift(worst):
    """Smallest log shift >= 0 with worst(shift) <= 1 (worst falls with shift)."""
    if worst(0.) <= 1.:
        return 0.
    low, high = 0., .05
    while worst(high) > 1.:
        low, high = high, high*2.
        if high > 8.:
            raise ValueError('dynamics budgets unreachable by uniform slow-down')
    for _ in range(40):
        middle = .5*(low+high)
        low, high = (middle, high) if worst(middle) > 1. else (low, middle)
    return high


def _scaled_worst(velocity, acceleration, jerk, dynamics, headroom=1.):
    """worst(shift) for a uniform slow-down of given v, a, j samples.

    A constant log-rate shift leaves L' and L'' unchanged, so v, a and j
    scale exactly by exp(-shift), exp(-2 shift), exp(-3 shift).
    """
    k = dynamics['drag_per_s']

    def worst(shift):
        c = math.exp(-shift)
        v, a, j = velocity*c, acceleration*c*c, jerk*c**3
        f = a+k*v
        f[:, 2] += GRAVITY
        ratios = _dynamics_ratios(f, j+k*a, dynamics)
        return max(float(np.max(value)) for value in ratios.values())/headroom
    return worst


def _fit_dynamics(trajectory, coefficients, dynamics):
    """Smallest uniform slow-down that satisfies every dense budget.

    0.1 % headroom covers curvature between the 201-point dense grid; the
    caller and tests re-check the final map independently.
    """
    dense = _dense_dynamics(trajectory, coefficients, dynamics)
    k = dynamics['drag_per_s']
    v = dense['velocity']
    a = dense['f']-k*v
    a[:, 2] -= GRAVITY
    j = dense['fdot']-k*a
    return _bisect_shift(_scaled_worst(v, a, j, dynamics, headroom=.999))


def dynamics_report(sampled, dynamics, threshold_deg=None):
    """Feed-forward tilt/thrust/tilt-rate summary from uniform time samples.

    sampled: FixedTrajectory.sample() dicts.  Works for the original and the
    retimed reference alike, so previews can compare both.
    """
    v = np.array([s['velocity'] for s in sampled])
    a = np.array([s['acceleration'] for s in sampled])
    j = np.array([s['jerk'] for s in sampled])
    k = dynamics['drag_per_s']
    f = a+k*v
    f[:, 2] += GRAVITY
    fdot = j+k*a
    tilt = np.degrees(np.arctan2(np.hypot(f[:, 0], f[:, 1]), f[:, 2]))
    rate = np.degrees(np.linalg.norm(np.cross(f, fdot), axis=1)/np.sum(f*f, axis=1))
    threshold = dynamics['feedforward_tilt_deg'] if threshold_deg is None else threshold_deg
    return dict(tilt_p50_deg=float(np.percentile(tilt, 50)), tilt_p95_deg=float(np.percentile(tilt, 95)),
                tilt_max_deg=float(tilt.max()), tilt_over_fraction=float(np.mean(tilt > threshold+1e-6)),
                tilt_threshold_deg=float(threshold),
                tilt_rate_p95_dps=float(np.percentile(rate, 95)), tilt_rate_max_dps=float(rate.max()),
                thrust_max_g=float(np.max(np.linalg.norm(f, axis=1))/GRAVITY),
                speed_max_mps=float(np.max(np.linalg.norm(v, axis=1))),
                acceleration_max_mps2=float(np.max(np.linalg.norm(a, axis=1))),
                jerk_max_mps3=float(np.max(np.linalg.norm(j, axis=1))))


def _optimize(trajectory, node_limits, groups, max_velocity, max_acceleration, dynamics=None):
    from scipy.interpolate import CubicSpline
    from scipy.optimize import minimize

    knots = np.array([0.]+list(trajectory._ends))
    keys, indices = {}, []
    for key in groups:
        if key not in keys:
            keys[key] = len(keys)
        indices.append(keys[key])
    mapping = np.zeros((len(knots), len(keys)))
    mapping[np.arange(len(knots)), indices] = 1.
    basis = CubicSpline(knots, mapping, bc_type='natural')
    windows = _cap_windows(node_limits, knots)
    qnodes, qweights = np.polynomial.legendre.leggauss(30)
    nodes = np.concatenate([a+(qnodes+1.)*.5*d for a, d in zip(trajectory._starts, trajectory.durations)])
    weights = np.concatenate([qweights*.5*d for d in trajectory.durations])
    bq = basis(nodes)
    samples = np.unique(np.concatenate([np.linspace(a, a+d, 31)
                        for a, d in zip(trajectory._starts, trajectory.durations)]))
    b, b1 = basis(samples), basis(samples, 1)
    v, a = _evaluate(trajectory, samples)
    if dynamics is not None:
        # Thrust budgets on every other mesh point keep SLSQP small on 9-lap
        # legs; the dense 201-point fit afterwards closes the gaps.
        thin = np.arange(0, len(samples), 2)
        bt, bt1, bt2 = b[thin], b1[thin], basis(samples[thin], 2)
        vt, at = v[thin], a[thin]
        d3, = _evaluate(trajectory, samples[thin], (3,))
    cap = np.full(len(samples), max_velocity)
    for window in windows:
        mask = (samples >= window['tau0']) & (samples <= window['tau1'])
        cap[mask] = np.minimum(cap[mask], window['cap'])
    v2 = np.sum(v*v, axis=1)/cap**2

    smoothing = 0. if dynamics is None else dynamics['smoothing']
    if smoothing:
        # Penalize log-rate slope per mean segment (dimensionless); scaled by
        # the base duration so `smoothing` trades seconds for evenness.
        mean_segment = trajectory.base_duration/len(trajectory.durations)
        bq1 = basis(nodes, 1)*mean_segment
        penalty = smoothing*np.einsum('i,ij,ik->jk', weights, bq1, bq1)

    def objective(x):
        weighted = weights*np.exp(-np.einsum('ij,j->i', bq, x))
        value, gradient = float(weighted.sum()), -np.einsum('ij,i->j', bq, weighted)
        if smoothing:
            value += float(x @ penalty @ x)
            gradient = gradient+2.*penalty @ x
        return value, gradient

    if dynamics is not None:
        tan2 = math.tan(math.radians(dynamics['feedforward_tilt_deg']))**2
        thrust2 = (dynamics['max_thrust_g']*GRAVITY)**2
        lift = dynamics['min_thrust_g']*GRAVITY
        omega2 = math.radians(dynamics['tilt_rate_dps'])**2
        drag = dynamics['drag_per_s']

    def dynamic_constraints(x):
        """Tilt, thrust, lift and tilt-rate budgets with analytic Jacobians."""
        lograte = bt @ x
        slope, curvature = bt1 @ x, bt2 @ x
        v, a, b, b1, b2 = vt, at, bt, bt1, bt2
        vel, acc, jerk, f, fdot = _thrust_terms(v, a, d3, lograte, slope, curvature, dynamics)
        r = np.exp(lograte)
        s1 = slope[:, None, None]
        B, B1, B2 = b[:, None, :], b1[:, None, :], b2[:, None, :]
        dv = vel[:, :, None]*B
        da = 2.*acc[:, :, None]*B+(r**2)[:, None, None]*v[:, :, None]*B1
        dj = (3.*jerk[:, :, None]*B+(r**3)[:, None, None]*(
            (3.*a[:, :, None]+4.*s1*v[:, :, None])*B1+v[:, :, None]*B2))
        df = da+drag*dv
        dfdot = dj+drag*da
        fz = f[:, 2]
        h2 = f[:, 0]**2+f[:, 1]**2
        dh2 = 2.*(f[:, 0, None]*df[:, 0]+f[:, 1, None]*df[:, 1])
        q_tilt = h2/(tan2*fz**2)
        dq_tilt = dh2/(tan2*fz**2)[:, None]-2.*(q_tilt/fz)[:, None]*df[:, 2]
        n2 = np.sum(f*f, axis=1)
        dn2 = 2.*np.einsum('ic,ick->ik', f, df)
        q_thrust, dq_thrust = n2/thrust2, dn2/thrust2
        q_lift = (lift/fz)**2
        dq_lift = -2.*(q_lift/fz)[:, None]*df[:, 2]
        cross = np.cross(f, fdot)
        dcross = (np.cross(df, fdot[:, :, None], axisa=1, axisb=1, axisc=1) +
                  np.cross(f[:, :, None], dfdot, axisa=1, axisb=1, axisc=1))
        c2 = np.sum(cross*cross, axis=1)
        dc2 = 2.*np.einsum('ic,ick->ik', cross, dcross)
        n4 = n2*n2
        q_rate = c2/(omega2*n4)
        dq_rate = dc2/(omega2*n4)[:, None]-(q_rate/n2)[:, None]*2.*dn2
        # Squared ratios grow like exp(6 L); log form log(1+e)-log(q+e) has
        # the same zero set but is close to linear in L, which keeps SLSQP's
        # line search well conditioned on near-stop segments.
        q = np.concatenate([q_tilt, q_thrust, q_lift, q_rate])
        dq = np.concatenate([dq_tilt, dq_thrust, dq_lift, dq_rate], axis=0)
        return math.log1p(_LOG_EPS)-np.log(q+_LOG_EPS), -dq/(q+_LOG_EPS)[:, None]

    def constraints(x):
        lograte = np.einsum('ij,j->i', b, x)
        slope = np.einsum('ij,j->i', b1, x)
        rate2 = np.exp(2.*lograte)
        u = a+v*slope[:, None]
        squared = np.sum(u*u, axis=1)
        speed = rate2*v2
        acc = rate2**2*squared/max_acceleration**2
        jacv = -2.*speed[:, None]*b
        jaca = -(rate2**2/max_acceleration**2)[:, None]*(4.*squared[:, None]*b+
                    2.*np.sum(u*v, axis=1)[:, None]*b1)
        values, jacobian = np.concatenate([1.-speed, 1.-acc]), np.concatenate([jacv, jaca], axis=0)
        if dynamics is not None:
            extra, extra_jacobian = dynamic_constraints(x)
            values = np.concatenate([values, extra])
            jacobian = np.concatenate([jacobian, extra_jacobian], axis=0)
        return values, jacobian

    initial = np.zeros(len(keys))
    for window in windows:
        for node in range(window['first'], window['last']+1):
            peak = max(float(np.linalg.norm(_evaluate(trajectory, np.array([knots[node]]))[0][0])), 1e-15)
            initial[indices[node]] = min(initial[indices[node]], math.log(min(1., window['cap']/peak)))
    if dynamics is not None:
        # Start SLSQP from a feasible uniform slow-down of the original timing.
        vel0, acc0, jerk0, _, _ = _thrust_terms(vt, at, d3, bt @ initial, bt1 @ initial,
                                                bt2 @ initial, dynamics)
        lograte0 = b @ initial
        rate2 = np.exp(2.*lograte0)
        u0 = a+v*(b1 @ initial)[:, None]
        speed0 = float(np.max(rate2*v2))
        accel0 = float(np.max(rate2**2*np.sum(u0*u0, axis=1)))/max_acceleration**2
        thrust_worst = _scaled_worst(vel0, acc0, jerk0, dynamics)
        initial = initial-_bisect_shift(lambda shift: max(
            speed0*math.exp(-2.*shift), accel0*math.exp(-4.*shift), thrust_worst(shift)))
    def solve(start):
        # Solver iterations are preparation work, not flight timeouts/retries.
        return minimize(lambda x: objective(x)[0], start, jac=lambda x: objective(x)[1],
            method='SLSQP', bounds=[(-6., 3.)]*len(keys),
            constraints=[dict(type='ineq', fun=lambda x: constraints(x)[0], jac=lambda x: constraints(x)[1])],
            options=dict(ftol=1e-8, maxiter=160, disp=False))

    def feasible(x):
        return np.isfinite(x).all() and np.min(constraints(x)[0]) >= -1e-6

    result = solve(initial)
    if not np.isfinite(result.x).all():
        raise ValueError('non-finite local timing solution')
    solution, solver_fallback, retries = result.x, None, 0
    if dynamics is not None and not (result.success and feasible(result.x)):
        # SLSQP line-search stops depend on the start point (2026-11-19: most
        # takeoff positions on two field routes). Retry from a few other starts
        # and keep the fastest feasible stop; only if none is feasible fall back
        # to the feasible uniform start. Never rescale an infeasible stop.
        best = result.x if feasible(result.x) else None
        for start in [initial+shift for shift in (.15, .3, .5)]+[.5*initial, np.zeros(len(keys))]:
            retries += 1
            trial = solve(start)
            if feasible(trial.x) and (best is None or objective(trial.x)[0] < objective(best)[0]):
                best, result = trial.x, trial
                if trial.success:
                    break
        if best is None:
            solution, solver_fallback = initial, 'uniform_slowdown'
        else:
            solution = best
    spline = CubicSpline(knots, np.einsum('ij,j->i', mapping, solution), bc_type='natural')
    coefficients = spline.c[::-1].T*np.asarray(trajectory.durations)[:, None]**np.arange(4)
    extrema = _extrema(trajectory, coefficients, windows, max_velocity, max_acceleration)
    # Normalization uses the original limits, including between-mesh extrema.
    scale = extrema['scale']*(1.+1e-9)
    coefficients[:, 0] -= math.log(scale)
    dynamics_shift = 0.
    if dynamics is not None:
        # Dense re-check of the thrust budgets between optimization samples.
        dynamics_shift = _fit_dynamics(trajectory, coefficients, dynamics)
        if dynamics_shift:
            dynamics_shift *= 1.+1e-9
            coefficients[:, 0] -= dynamics_shift
            # A uniform shift scales speed/acceleration peaks exactly.
            c = math.exp(-dynamics_shift)
            extrema['speed'] *= c
            extrema['acceleration'] *= c*c
            extrema['gate_peaks'] = [peak*c for peak in extrema['gate_peaks']]
    timing = LocalTimeMap.prepare(knots, coefficients)
    if not (np.isfinite(timing.elapsed).all() and np.all(np.diff(timing.elapsed) > 0.)):
        raise ValueError('non-finite local time map')
    metadata = dict(method='local_log_rate_spline', applied=True,
                    baseline_duration_s=trajectory.duration, duration_s=timing.duration,
                    solver_converged=bool(result.success), iterations=int(result.nit), solver_retries=retries,
                    extrema_normalization=scale, spatial_curve_unchanged=True)
    if dynamics is not None:
        dense = _dense_dynamics(trajectory, coefficients, dynamics)
        if solver_fallback:
            metadata['solver_fallback'] = solver_fallback
        metadata.update(constraint_model='thrust_direction',
            dynamics_limits=dict(dynamics), dynamics_normalization=math.exp(dynamics_shift),
            dynamics_peak_ratios={k: float(np.max(v)) for k, v in dense['ratios'].items()})
    extrema['speed'] /= scale
    extrema['acceleration'] /= scale**2
    extrema['gate_peaks'] = [peak/scale for peak in extrema['gate_peaks']]
    return timing, metadata, extrema, windows


def describe_dynamics(detail):
    """One preview line for thrust-direction timing, or None for the old model."""
    if not detail or detail.get('constraint_model') != 'thrust_direction':
        return None
    limits, peaks = detail['dynamics_limits'], detail['dynamics_peak_ratios']
    tilt = math.degrees(math.atan(peaks['tilt']*math.tan(math.radians(limits['feedforward_tilt_deg']))))
    text = ('推力方向约束：前馈倾角 ≤ %.0f°（峰值 %.1f°），倾转角速度 ≤ %.0f°/s（峰值 %.0f°/s），'
            '总推力峰值 %.2f g，阻力按 %.2f·v 计入' % (
                limits['feedforward_tilt_deg'], tilt, limits['tilt_rate_dps'],
                peaks['tilt_rate']*limits['tilt_rate_dps'], peaks['thrust']*limits['max_thrust_g'],
                limits['drag_per_s']))
    if detail.get('slower_than_original'):
        text += '；该预算下比原配时慢，仍按预算采用'
    return text+'。'


def retime_loaded_leg(trajectory, event_caps, event_groups, max_velocity, max_acceleration,
                      dynamics=None):
    """Return a faster reference or the unchanged original; never reject flight.

    Used for a delivery-route leg containing loaded laps and explicitly chosen
    ordinary-route timing comparisons.  dynamics (from dynamics_limits) adds
    the thrust-direction budgets; None reproduces the deployed optimizer. Later drop legs, HOME, single-lap
    delivery and both drop-test entries do not call it.
    """
    from .fixed_trajectory import FixedTrajectory

    if not trajectory.durations:
        return trajectory
    limits = [None]*len(trajectory.nodes)
    groups = [('origin', i) for i in range(len(trajectory.nodes))]
    assigned = set()
    for event, cap, group in zip(trajectory.events, event_caps, event_groups):
        node = event['node_index']
        if cap is not None:
            limits[node] = min(cap, limits[node] or math.inf)
        if node not in assigned:
            groups[node] = group
            assigned.add(node)
    metadata = dict(trajectory._metadata)
    try:
        timing, detail, extrema, windows = _optimize(trajectory, limits, groups, max_velocity,
                                                     max_acceleration, dynamics)
        if dynamics is None and timing.duration >= trajectory.duration:
            detail.update(applied=False, reason='original_reference_is_faster')
        else:
            if timing.duration >= trajectory.duration:
                # Thrust budgets chosen explicitly: keep them even if slower.
                detail.update(slower_than_original=True)
            events = [dict(event, elapsed_s=timing.time_at_base(
                0. if event['node_index'] == 0 else trajectory._ends[event['node_index']-1]))
                for event in trajectory.events]
            for key in ('warp_time_scale', 'limited_peak_velocity_mps', 'gate_windows'):
                metadata.pop(key, None)
            metadata.update(local_retiming=detail, time_warp=True,
                max_velocity_mps=extrema['speed'], max_acceleration_mps2=extrema['acceleration'],
                extrema='stationary_roots_and_dense_check',
                event_velocity_limits_mps=list(event_caps),
                gate_windows=[dict(start_s=timing.time_at_base(w['tau0']),
                                   end_s=timing.time_at_base(w['tau1']), cap_mps=w['cap'], peak_mps=peak)
                              for w, peak in zip(windows, extrema['gate_peaks'])])
            if windows:
                metadata['limited_peak_velocity_mps'] = max(extrema['gate_peaks'])
            return FixedTrajectory(trajectory.nodes, trajectory.durations, trajectory.coefficients,
                                   events, metadata, retiming=timing.value)
    except Exception as exc:
        detail = dict(method='local_log_rate_spline', applied=False,
                      reason='original_reference_fallback', diagnostic=str(exc))
    # Preserve all original samples/events/caps when optimization adds no gain
    # or its optional dependency is absent. Report the fallback in saved logs.
    metadata['local_retiming'] = detail
    return FixedTrajectory(trajectory.nodes, trajectory.durations, trajectory.coefficients,
                           trajectory.events, metadata, warp=trajectory.warp)
