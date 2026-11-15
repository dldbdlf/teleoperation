"""EX00: validate that a synthetic (or, eventually, real) trajectory file
conforms to the Observation/HumanBelief contract documented in README.md.

Usage:
    python3 validate_ex00_contract.py <file.npz>
    python3 validate_ex00_contract.py <directory>   # validates every .npz found

Exits 0 and prints PASS per file if every check passes, else prints the
specific failing checks and exits 1.
"""
import sys
from pathlib import Path

import numpy as np

REQUIRED_FIELDS = {
    'obs_t': ('f', (None,)),
    'obs_W': ('f', (None, 3)),
    'obs_R': ('f', (None, 3, 3)),
    'obs_E': ('f', (None, 3)),
    'obs_psi': ('f', (None,)),
    'obs_wrist_visible': ('b', (None,)),
    'obs_elbow_visible': ('b', (None,)),
    'belief_t': ('f', (None,)),
    'belief_wrist_T': ('f', (None, 4, 4)),
    'belief_W': ('f', (None, 3)),
    'belief_wrist_cov': ('f', (None, 6, 6)),
    'belief_psi_mean': ('f', (None,)),
    'belief_psi_sigma': ('f', (None,)),
    'belief_psi_kappa': ('f', (None,)),
    'belief_occlusion_state': ('i', (None,)),
    'belief_age_wrist_ms': ('f', (None,)),
    'belief_age_elbow_ms': ('f', (None,)),
}

OCCLUSION_CODES = {0, 1, 2}  # VISIBLE, ELBOW_HIDDEN, WRIST_HIDDEN


def check_kind(dtype, want):
    kind = dtype.kind
    if want == 'f':
        return kind in ('f',)
    if want == 'b':
        return kind in ('b',)
    if want == 'i':
        return kind in ('i', 'u')
    return False


def validate(path):
    errors = []
    try:
        d = np.load(path, allow_pickle=True)
    except Exception as e:
        return [f'failed to load: {e}']

    # 1. required fields present with right dtype kind / rank
    for name, (kind, shape) in REQUIRED_FIELDS.items():
        if name not in d.files:
            errors.append(f'missing field: {name}')
            continue
        arr = d[name]
        if not check_kind(arr.dtype, kind):
            errors.append(f'{name}: expected dtype kind {kind!r}, got {arr.dtype}')
        if len(arr.shape) != len(shape):
            errors.append(f'{name}: expected rank {len(shape)}, got shape {arr.shape}')
            continue
        for ax, want in enumerate(shape):
            if want is not None and arr.shape[ax] != want:
                errors.append(f'{name}: axis {ax} expected {want}, got {arr.shape}')

    if errors:
        return errors  # shape issues make the rest unsafe to run

    n_obs = d['obs_t'].shape[0]
    n_bel = d['belief_t'].shape[0]
    if n_obs != n_bel:
        errors.append(f'obs_t length {n_obs} != belief_t length {n_bel}')

    # 2. timestamps monotonic non-decreasing
    for name in ('obs_t', 'belief_t'):
        t = d[name]
        if t.size > 1 and np.any(np.diff(t) < -1e-9):
            errors.append(f'{name}: not monotonic non-decreasing')

    # 3. psi in [-pi, pi] where defined (NaN allowed when occluded)
    for name in ('obs_psi', 'belief_psi_mean'):
        psi = d[name]
        finite = psi[np.isfinite(psi)]
        if finite.size and (finite.min() < -np.pi - 1e-6 or finite.max() > np.pi + 1e-6):
            errors.append(f'{name}: value(s) outside [-pi, pi]: min={finite.min():.4f} max={finite.max():.4f}')

    # 4. occlusion-NaN consistency: hidden -> NaN, visible -> finite
    wv, ev = d['obs_wrist_visible'], d['obs_elbow_visible']
    for name, vis in (('obs_W', wv), ('obs_R', wv), ('obs_E', ev), ('obs_psi', ev)):
        arr = d[name]
        nan_mask = ~np.isfinite(arr).reshape(arr.shape[0], -1).all(axis=1)
        if np.any(nan_mask & vis):
            errors.append(f'{name}: NaN while marked visible')
        if np.any(~nan_mask & ~vis):
            errors.append(f'{name}: finite while marked hidden (expected NaN)')
    if np.any(~wv & ev):
        errors.append('obs_elbow_visible=True while obs_wrist_visible=False (wrist hidden must imply elbow hidden)')

    # 5. belief_wrist_cov symmetric and positive semi-definite
    cov = d['belief_wrist_cov']
    asym = np.abs(cov - np.transpose(cov, (0, 2, 1))).max()
    if asym > 1e-6:
        errors.append(f'belief_wrist_cov: not symmetric, max asymmetry {asym:.2e}')
    sample_idx = np.linspace(0, cov.shape[0] - 1, min(10, cov.shape[0])).astype(int)
    for i in sample_idx:
        eigvals = np.linalg.eigvalsh(cov[i])
        if eigvals.min() < -1e-9:
            errors.append(f'belief_wrist_cov[{i}]: not PSD, min eigenvalue {eigvals.min():.2e}')
            break

    # 6. rotation matrices orthogonal (R^T R = I, det = +1)
    for name in ('obs_R',):
        R = d[name]
        finite_idx = np.where(np.isfinite(R).reshape(R.shape[0], -1).all(axis=1))[0]
        check_idx = finite_idx[np.linspace(0, len(finite_idx) - 1, min(10, len(finite_idx))).astype(int)] \
            if len(finite_idx) else []
        for i in check_idx:
            Ri = R[i]
            orth_err = np.abs(Ri.T @ Ri - np.eye(3)).max()
            det = np.linalg.det(Ri)
            if orth_err > 1e-4:
                errors.append(f'{name}[{i}]: not orthogonal, max error {orth_err:.2e}')
            if abs(det - 1.0) > 1e-3:
                errors.append(f'{name}[{i}]: det={det:.4f}, expected +1')

    # 7. occlusion_state codes valid, ages non-negative
    codes = set(np.unique(d['belief_occlusion_state']).tolist())
    if not codes.issubset(OCCLUSION_CODES):
        errors.append(f'belief_occlusion_state: unexpected codes {codes - OCCLUSION_CODES}')
    for name in ('belief_age_wrist_ms', 'belief_age_elbow_ms'):
        if np.any(d[name] < -1e-6):
            errors.append(f'{name}: negative age found')

    return errors


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    target = Path(sys.argv[1])
    files = sorted(target.rglob('*.npz')) if target.is_dir() else [target]
    if not files:
        print(f'no .npz files found under {target}')
        sys.exit(2)

    all_ok = True
    for f in files:
        errs = validate(f)
        if errs:
            all_ok = False
            print(f'FAIL {f}')
            for e in errs:
                print(f'  - {e}')
        else:
            print(f'PASS {f}')

    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
