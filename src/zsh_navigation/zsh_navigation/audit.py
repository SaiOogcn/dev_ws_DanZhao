"""Audit route geometry without importing or starting ROS."""
import argparse
import json
from pathlib import Path

from .race_course import audit_course, build_course


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--share', type=Path)
    args = parser.parse_args()
    if args.share is None:
        source = Path(__file__).resolve().parents[1]
        if (source / 'config/full_race/parking_entry.json').exists():
            args.share = source
        else:
            from ament_index_python.packages import get_package_share_directory
            args.share = Path(get_package_share_directory('zsh_navigation'))
    failed = False
    for direction in ('CW', 'CCW'):
        report = audit_course(build_course(args.share, direction))
        failed |= bool(report['illegal_reference_poses'] or report['stage_gaps'])
        print(json.dumps({'direction': direction, **report}, ensure_ascii=False))
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
