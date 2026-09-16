"""Axial-only rear compliance extension of the supplied notebook.

Run: python rear_toe_compliance.py
Requires numpy, pandas, matplotlib and scipy. No BobSim execution.
Fixed inboards, rigid upright, uniform EA/L links, small displacement.
The pullrod retains the notebook's idealized direct-upright attachment.
"""
from pathlib import Path
import ast
import csv
import hashlib
import json
import numpy as np
from scipy.optimize import root
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
N_PER_LBF = 4.4482216152605
NM_PER_LBFIN = N_PER_LBF * 0.0254


def run():
    nb_path = HERE / 'truss_solver.ipynb'
    hp_path = HERE / 'hardpoints.txt'
    nb = json.loads(nb_path.read_text(encoding='utf-8'))
    ns = {}
    # Only reviewed definition/setup cells; do not run load sweeps or outputs.
    for i in [0, 1]:
        exec(''.join(nb['cells'][i]['source']), ns)
    ns['hardpoints'] = ns['get_hardpoints'](hp_path)
    for i in [3, 4, 5, 6, 7, 8, 11, 12, 26, 28]:
        exec(''.join(nb['cells'][i]['source']), ns)
    links = ns['links_rear']
    A, names = ns['build_equilibrium_matrix'](
        links, ns['unit_vectors_rear'], ns['moment_arms_rear'])
    saved_text = ''.join(nb['cells'][15]['outputs'][0]['data']['text/plain'])
    saved_A = np.array(ast.literal_eval(saved_text[6:-1]))
    saved_error = float(np.max(abs(A-saved_A)))
    assert saved_error < 6e-9, 'Hardpoints do not match saved notebook matrix'
    lengths = np.array([ns['compute_length'](links)[n] for n in names])
    props = ns['rear_tube_properties']
    k = np.array([props[n]['E'] * props[n]['A'] for n in names]) / lengths
    # A maps compression-positive forces to upright wrench. A.T maps
    # upright translation/rotation q to positive member extension.
    K = (A*k) @ A.T
    wrench = np.array([0., 0., 0., 0., 0., 42/NM_PER_LBFIN])
    F = np.linalg.solve(A, -wrench)
    extension = -F/k
    q = np.linalg.solve(A.T, extension)
    q_stiffness = np.linalg.solve(K, wrench)
    np.testing.assert_allclose(q, q_stiffness, rtol=1e-9, atol=1e-12)
    # Opposite corner reflected across vehicle centre plane, same global Mz.
    mirror = np.array([1., -1., 1.])
    mirrored_links = {n: (np.array(o)*mirror, np.array(i)*mirror)
                      for n, (o, i) in links.items()}
    Am, _ = ns['build_equilibrium_matrix'](mirrored_links,
        ns['compute_unit_vectors'](mirrored_links),
        ns['compute_moment_arms'](mirrored_links, np.array(ns['R_CP'])*mirror))
    qm = np.linalg.solve((Am*k)@Am.T, wrench)
    np.testing.assert_allclose(qm[5], q[5], rtol=1e-9)
    np.testing.assert_allclose(np.linalg.solve(K, -wrench), -q, rtol=1e-9, atol=1e-12)
    assert np.max(abs(A @ F+wrench)) < 1e-9
    assert abs(wrench@q - np.sum(k*extension**2)) < 1e-10
    unit_wrench = np.array([0., 0., 0., 0., 0., 1.])
    unit_tension = np.linalg.solve(A, unit_wrench)
    contributions = np.degrees(unit_tension * extension)
    toe = float(np.degrees(q[5]))
    assert abs(contributions.sum()-toe) < 1e-10
    # Independent finite-rotation compatibility, using the linear extensions.
    cp = np.array(ns['R_CP'])
    offsets = np.array([links[n][0] for n in names])-cp
    anchors = np.array([links[n][1] for n in names])
    def compat(x):
        points = cp+x[:3]+Rotation.from_rotvec(x[3:]).apply(offsets)
        return np.linalg.norm(points-anchors, axis=1)-lengths-extension
    finite = root(compat, q, tol=1e-10)
    assert np.max(abs(compat(finite.x))) < 1e-9
    heading = Rotation.from_rotvec(finite.x[3:]).apply([1., 0., 0.])
    finite_toe = float(np.degrees(np.arctan2(heading[1], heading[0])))
    pull_out, pull_in = map(np.array, links['PULLROD'])
    pull_u = (pull_out-pull_in)/np.linalg.norm(pull_out-pull_in)
    miss = float(np.linalg.norm(np.cross(pull_out-np.array(links['UCA_FORE'][0]), pull_u)))
    rows = [dict(link=n, length_in=float(lengths[i]), stiffness_lbf_per_in=float(k[i]),
                 force_N_compression_positive=float(F[i]*N_PER_LBF),
                 extension_mm=float(extension[i]*25.4), toe_contribution_deg=float(contributions[i]))
            for i, n in enumerate(names)]
    out = HERE/'rear_toe_results'
    out.mkdir(exist_ok=True)
    with (out/'member_results.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    result = dict(moment_per_wheel_Nm=42, toe_deg=toe, toe_deg_per_Nm=toe/42,
                  rotational_stiffness_Nm_per_deg=42/toe,
                  equal_split_42Nm_axle_per_wheel_toe_deg=toe/2,
                  q_translation_in_rotation_rad=q.tolist(), members=rows,
                  validation=dict(saved_matrix_max_difference=saved_error,
                    equilibrium_residual=float(np.max(abs(A@F+wrench))),
                    mirrored_corner_yaw_deg=float(np.degrees(qm[5])),
                    finite_compatibility_toe_deg=finite_toe,
                    finite_compatibility_residual_in=float(np.max(abs(compat(finite.x)))),
                    pullrod_line_miss_UCA_in=miss),
                  inputs_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [nb_path,hp_path]},
                  limitations='Axial tube stretch only; all six links use notebook steel properties. Fixed inboards and pullrod inner point; rigid upright. No joints, bearings, chassis, rim, tire, arm bending, rocker/spring compliance, preload geometric stiffness or complete axle coupling. Geometry matches saved notebook, not current vehicle YAML. Toe is incremental yaw of initially x-aligned wheel.')
    (out/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(1,2,figsize=(10,4.3),layout='constrained')
    moments=np.linspace(-42,42,85)
    axes[0].plot(moments,moments*toe/42, color='#d36128')
    axes[0].scatter([21,42],[toe/2,toe],color='#223a5e')
    axes[0].set(xlabel='Aligning moment at one rear wheel (N m)',ylabel='Incremental wheel yaw (deg)',title='Steel axial-link model')
    axes[0].grid(alpha=.25)
    axes[1].barh(names,contributions,color='#223a5e')
    axes[1].set(xlabel='Toe contribution at +42 N m (deg)',title='Virtual-work decomposition')
    fig.suptitle('Notebook rear geometry | fixed inboards | rigid upright')
    fig.savefig(out/'rear_toe_compliance.png',dpi=180)
    plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    run()
