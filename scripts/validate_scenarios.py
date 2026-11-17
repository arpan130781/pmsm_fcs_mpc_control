#!/usr/bin/env python3
"""Run complete verification of all 13 SIL fault scenarios.

Exports machine-readable metrics to docs/results/validation/scenarios_summary.json
and docs/results/validation/scenarios_summary.csv.
"""
import csv, json, os, sys, yaml
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, ROOT)

from simulation.harness import config
from simulation.harness.sil_runner import SILRunner
from simulation.analysis import metrics

def run_all():
    yaml_path = os.path.join(ROOT, 'simulation', 'fault_injection', 'fault_scenarios.yaml')
    with open(yaml_path) as f:
        all_scenarios = yaml.safe_load(f)

    results_dir = os.path.join(ROOT, 'docs', 'results', 'validation')
    os.makedirs(results_dir, exist_ok=True)

    summary_records = []
    all_passed = True

    print(f"================================================================================")
    print(f"  RUNNING ALL {len(all_scenarios)} FAULT SCENARIOS (SIL FULL VERIFICATION)")
    print(f"================================================================================\n")

    for name, conf in all_scenarios.items():
        print(f"--- Running {name} ---")
        runner = SILRunner(config_sim=config.SIM_PARAMS, fault_scenario=name)
        res = runner.run()

        # Extract criteria & ground truth
        sc_conf = runner.scenario_config or {}
        exp_outcome = sc_conf.get('expected_outcome', {})
        criteria = exp_outcome.get('criteria', {
            'max_angle_error': 15.0,
            'angle_error_rms': 5.0,
            'speed_ripple_ptp': 350.0,
            'mean_speed': 20.0
        })

        onset = exp_outcome.get('fault_onset_time', float('inf'))
        expected_ch = exp_outcome.get('expected_fault_channel', 0)
        expected_mode = exp_outcome.get('expected_mode_after_fault', 2)

        ref_speed = sc_conf.get('speed_reversal', {}).get('new_speed_rpm', sc_conf.get('speed_ref_rpm', config.SIM_PARAMS['speed_ref_rpm']))
        if 'slow_ramp' in sc_conf:
            rate = sc_conf['slow_ramp']['rate_rpm_per_s']
            duration = config.SIM_PARAMS['total_time'] - sc_conf['slow_ramp']['onset_time']
            ref_speed = ref_speed + rate * duration

        trans_time = 0.0
        if 'speed_reversal' in sc_conf: trans_time = sc_conf['speed_reversal'].get('onset_time', 0.0)
        elif 'load_step' in sc_conf: trans_time = sc_conf['load_step'].get('onset_time', 0.0)
        elif 'slow_ramp' in sc_conf: trans_time = sc_conf['slow_ramp'].get('onset_time', 0.0)

        eval_res = metrics.evaluate_scenario(
            res, criteria, onset, ref_speed, start_time=2.0,
            expected_fault_channel=expected_ch,
            expected_mode_after_fault=expected_mode,
            disturbance_duration=sc_conf.get('duration', 0.0),
            transition_time=trans_time
        )

        sc_passed = all(item['pass'] for item in eval_res.values())
        if not sc_passed:
            all_passed = False

        # Additional detailed metrics
        end_mask = res.time >= 2.0
        mean_spd_actual = float(np.mean(res.speed_rpm[end_mask]))
        spd_err = abs(mean_spd_actual - ref_speed)
        ptp_spd = float(np.ptp(res.speed_rpm[end_mask]))
        ang_rms = float(metrics.compute_angle_error_rms(res, start_time=2.0))
        max_ang = float(metrics.compute_max_angle_error(res, start_time=2.0))
        lat_dict = metrics.compute_detection_latency(res, onset)
        latency = lat_dict['latency_ms']
        recon_dict = metrics.compute_current_reconstruction_error(res, onset)
        recon_rms = recon_dict['rms']

        post_modes = res.control_mode[res.time >= (onset if onset < float('inf') else 2.0)]
        actual_mode = int(post_modes[-1]) if len(post_modes) > 0 else int(res.control_mode[-1])

        post_channels = res.fault_channel[res.time >= (onset if onset < float('inf') else 2.0)]
        isolated_ch = int(post_channels[-1]) if len(post_channels) > 0 else 0

        rec = {
            'scenario': name,
            'passed': sc_passed,
            'fault_type': sc_conf.get('type', 'none'),
            'fault_channel': sc_conf.get('channel', 'none'),
            'fault_magnitude': sc_conf.get('offset', sc_conf.get('limit', sc_conf.get('magnitude', sc_conf.get('scale', 0.0)))),
            'onset_time': onset if onset < float('inf') else None,
            'ref_speed_rpm': ref_speed,
            'actual_mean_speed_rpm': round(mean_spd_actual, 2),
            'speed_error_rpm': round(spd_err, 2),
            'speed_ripple_ptp_rpm': round(ptp_spd, 2),
            'angle_rms_deg': round(ang_rms, 2),
            'max_angle_error_deg': round(max_ang, 2),
            'detection_latency_ms': round(latency, 2) if latency >= 0 else None,
            'expected_channel': expected_ch,
            'isolated_channel': isolated_ch,
            'reconstruction_error_rms_A': round(recon_rms, 3),
            'expected_mode': expected_mode,
            'actual_mode': actual_mode,
            'criteria': {k: {'pass': bool(v['pass']), 'value': round(float(v['value']), 3)} for k, v in eval_res.items()}
        }
        summary_records.append(rec)

        status_str = "PASS" if sc_passed else "FAIL"
        print(f"  Result: {status_str} | Mode: Exp {expected_mode} / Act {actual_mode} | Latency: {latency:.2f} ms | MaxAng: {max_ang:.2f} deg | SpdErr: {spd_err:.2f} RPM")
        for k, v in eval_res.items():
            print(f"    - {k:22s}: {'PASS' if v['pass'] else 'FAIL'} (value={v['value']:.2f})")
        print()

    # Save to JSON
    json_path = os.path.join(results_dir, 'scenarios_summary.json')
    with open(json_path, 'w') as f:
        json.dump(summary_records, f, indent=2)
    print(f"Saved JSON summary: {json_path}")

    # Save to CSV
    csv_path = os.path.join(results_dir, 'scenarios_summary.csv')
    fieldnames = [
        'scenario', 'passed', 'fault_type', 'fault_channel', 'fault_magnitude',
        'onset_time', 'ref_speed_rpm', 'actual_mean_speed_rpm', 'speed_error_rpm',
        'speed_ripple_ptp_rpm', 'angle_rms_deg', 'max_angle_error_deg',
        'detection_latency_ms', 'expected_channel', 'isolated_channel',
        'reconstruction_error_rms_A', 'expected_mode', 'actual_mode'
    ]
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        for r in summary_records:
            writer.writerow(r)
    print(f"Saved CSV summary: {csv_path}")

    print(f"\nOVERALL RESULT: {'ALL SCENARIOS PASSED (13/13)' if all_passed else 'SOME SCENARIOS FAILED'}")
    return 0 if all_passed else 1

if __name__ == '__main__':
    sys.exit(run_all())
