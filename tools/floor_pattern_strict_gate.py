#!/usr/bin/env python3
import argparse, json
from pathlib import Path


def evaluate(pattern: dict) -> dict:
    failures = []
    if pattern.get('major_seam_count', 0) < 4:
        failures.append('need at least 4 supported primary seams')
    if pattern.get('lattice_inlier_count', 0) < 4 or pattern.get('lattice_run_length', 0) < 4:
        failures.append('need a 4-seam repeated lattice, not a couple of lines')
    if pattern.get('secondary_seam_count', 0) < 4:
        failures.append('need at least 4 retained cross-board seams')
    if pattern.get('butt_joint_confidence', 0.0) < 0.62:
        failures.append('cross-board butt-joint graph too weak')
    if pattern.get('butt_joint_lane_count', 0) < 3:
        failures.append('cross-board evidence must span at least 3 plank lanes')
    if pattern.get('butt_joint_axial_bucket_count', 0) < 3:
        failures.append('cross-board evidence must occupy at least 3 axial buckets')
    valid = len(failures) == 0 and bool(pattern.get('valid'))
    return {
        'valid': valid,
        'reason': 'hardwood_planks_pattern_verified_by_dense_primary_lattice_and_strict_butt_joint_graph' if valid else 'pattern_rejected_sparse_primary_family_or_weak_cross_board_pattern',
        'failure_reasons': failures,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description='Apply the strict hardwood-pattern gate to an existing floor-image-doctor JSON report')
    ap.add_argument('json_report', help='path to floor-image-doctor JSON report')
    args = ap.parse_args()
    data = json.loads(Path(args.json_report).read_text())
    pattern = data['detection_debug']['pattern']
    summary = {
        'input_report': str(Path(args.json_report).resolve()),
        'base_result': {
            'valid': pattern.get('valid'),
            'reason': pattern.get('reason'),
            'major_seam_count': pattern.get('major_seam_count'),
            'secondary_seam_count': pattern.get('secondary_seam_count'),
            'lattice_inlier_count': pattern.get('lattice_inlier_count'),
            'lattice_run_length': pattern.get('lattice_run_length'),
            'butt_joint_confidence': pattern.get('butt_joint_confidence'),
            'butt_joint_lane_count': pattern.get('butt_joint_lane_count'),
            'butt_joint_axial_bucket_count': pattern.get('butt_joint_axial_bucket_count'),
        },
        'strict_result': evaluate(pattern),
    }
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
