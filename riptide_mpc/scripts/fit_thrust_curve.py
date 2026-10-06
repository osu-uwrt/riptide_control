#!/usr/bin/env python3
"""Fit the hardware thrust coefficient K_T = a + b rpm of the propeller law |F| = K_T rho D^4 (rpm/60)^2.

Inputs (any mix):
  - calibrate_thruster CSVs (riptide_controllers; ~/thruster_cal_data/*.csv): columns dshot, rpm (measured),
    optional current/voltage, force (N), positive and/or negative commands in one file. Each direction of each
    file is tared by its reading at the lowest command (rpm ~ 0).
  - Blue Robotics performance spreadsheets (.xlsx, e.g. T200-Public-Performance-Data-10-20V-September-2019.xlsx):
    every sheet with PWM (us), RPM and Force (Kg f) columns; PWM above 1500 is forward.
Points under --min-force are dropped (load-cell noise). K_T is computed for every point and fitted with a
straight line in rpm per direction.

Prints the `hardware:` lines for the model YAML (propeller_diameter / thrust_coefficient_forward / _reverse).

    fit_thrust_curve.py T200-Public-Performance-Data-10-20V-September-2019.xlsx
    fit_thrust_curve.py data/thruster_cal_2023-10-10/*.csv
"""
import argparse
import csv
import re
import sys
import zipfile

import numpy as np

KGF = 9.80665


def load_csv(path):
    with open(path, newline='') as f:
        rows = list(csv.reader(f))
    header = [h.strip().lower() for h in rows[0]]
    cmd, rpm = header.index('dshot'), header.index('rpm')
    force = next(i for i, h in enumerate(header) if h.startswith('force'))
    data = np.array([[float(r[cmd]), float(r[rpm]), float(r[force])] for r in rows[1:] if r])
    runs = {}
    for sign in (1, -1):
        d = data[np.sign(data[:, 0]) == sign]
        if len(d) < 5:
            continue
        d = d[np.argsort(np.abs(d[:, 0]))]
        runs[sign] = (np.abs(d[:, 1]), np.abs(d[:, 2] - d[0, 2]))  # tare at the lowest command
    return runs


def load_xlsx(path):
    """Blue Robotics sheets, read with the standard library (no openpyxl)."""
    z = zipfile.ZipFile(path)
    names = z.namelist()
    strings = []
    if 'xl/sharedStrings.xml' in names:
        x = z.read('xl/sharedStrings.xml').decode()
        strings = [re.sub(r'<[^>]+>', '', s).strip().lower() for s in re.findall(r'<si>(.*?)</si>', x, re.S)]
    rows = []
    for name in sorted(n for n in names if re.match(r'xl/worksheets/sheet\d+\.xml$', n)):
        sheet = re.findall(r'<row [^>]*>(.*?)</row>', z.read(name).decode(), re.S)
        if not sheet:
            continue
        cell = r'<c r="([A-Z]+)\d+"([^>]*)>(?:<f>.*?</f>)?<v>([^<]*)</v>'
        header = {col: strings[int(v)] for col, attr, v in re.findall(cell, sheet[0]) if 't="s"' in attr}
        cols = {k: next((c for c, h in header.items() if h.startswith(k)), None) for k in ('pwm', 'rpm', 'force')}
        if None in cols.values():
            continue
        for row in sheet[1:]:
            v = {col: float(val) for col, attr, val in re.findall(cell, row) if 't="s"' not in attr}
            if all(c in v for c in cols.values()):
                rows.append((v[cols['pwm']], v[cols['rpm']], v[cols['force']] * KGF))
    data = np.array(rows)
    if not len(data):
        sys.exit(f'{path}: no sheet with PWM, RPM and Force columns')
    return {sign: (np.abs(data[m, 1]), np.abs(data[m, 2]))
            for sign, m in ((1, data[:, 0] > 1500), (-1, data[:, 0] < 1500))}


def thrust(k, rpm, scale):
    return (k[0] + k[1] * rpm) * scale * rpm * rpm


def inverse(k, force, scale):
    rpm = np.linspace(0, 6000, 60001)
    return np.interp(force, thrust(k, rpm, scale), rpm)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('files', nargs='+', help='calibrate_thruster .csv and/or Blue Robotics .xlsx')
    parser.add_argument('--min-force', type=float, default=0.3, help='N; drop points below this (default 0.3)')
    parser.add_argument('--diameter', type=float, default=0.076, help='propeller diameter, m (T200: 0.076)')
    parser.add_argument('--density', type=float, default=998.2,
                        help='water density of the data, kg/m^3; the MPC uses the model file water_density')
    args = parser.parse_args()

    scale = args.density * args.diameter ** 4 / 3600.0  # N per (K_T rpm^2)
    runs = {p: (load_xlsx(p) if p.lower().endswith('.xlsx') else load_csv(p)) for p in args.files}
    lines = [f'  propeller_diameter: {args.diameter:g}']
    for sign, name in ((1, 'forward'), (-1, 'reverse')):
        parts = [(p, *r[sign]) for p, r in runs.items() if sign in r]
        if not parts:
            sys.exit(f'no {name} data')
        rpm = np.concatenate([p[1] for p in parts])
        force = np.concatenate([p[2] for p in parts])
        keep = (force >= args.min_force) & (rpm > 0)
        kt = force[keep] / (scale * rpm[keep] ** 2)
        b, a = np.polyfit(rpm[keep], kt, 1)
        k = (a, b)
        if not (a > 0 and b >= 0):
            sys.exit(f'{name}: fit gave a={a:.4g}, b={b:.4g}; the MPC needs a > 0, b >= 0')
        err = inverse(k, force[keep], scale) - rpm[keep]
        probe = np.array([4.0, 12.0, 20.0])
        print(f'{name}: {keep.sum()} points from {len(parts)} source(s), '
              f'{force[keep].min():.2f}..{force.max():.1f} N, max {rpm.max():.0f} rpm; K_T {a:.4f} + {b:.3g} rpm; '
              f'RPM rms {np.sqrt(np.mean(err ** 2)):.0f}; rpm at 4/12/20 N '
              f'{np.round(inverse(k, probe, scale)).astype(int).tolist()}')
        lines.append(f'  thrust_coefficient_{name}: [{a:.4f}, {b:.3g}]')
    print('\n'.join(['', 'hardware:  # |F| = K_T rho D^4 (rpm/60)^2, K_T = a + b rpm, [a, b]', *lines]))


if __name__ == '__main__':
    main()
