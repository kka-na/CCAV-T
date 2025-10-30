#!/usr/bin/env python3
"""
Update ego_pose.yaml Midan section with CLM scenario points from UI yaml files
"""

import yaml

def load_points():
    """Load ego and target points from UI yaml files"""
    with open("../../ui/yamls/ego_point.yaml", "r") as f:
        ego_points = yaml.safe_load(f)

    with open("../../ui/yamls/target_point.yaml", "r") as f:
        target_points = yaml.safe_load(f)

    return ego_points, target_points


def update_ego_pose_yaml():
    """Update ego_pose.yaml with CLM scenario points"""

    # Load existing ego_pose.yaml
    with open("./yamls/ego_pose.yaml", "r") as f:
        config = yaml.safe_load(f)

    # Load points from UI yaml files
    ego_points, target_points = load_points()

    # Create new Midan section
    midan_config = {}

    # Process each CLM scenario (1-6) and each speed variant (slower, same, faster)
    for clm_num in range(1, 7):
        for variant in ['slower', 'same', 'faster']:
            key = f"CLM{clm_num}_{variant}"

            # Get points (format: [x, y])
            ego_point = ego_points.get(key, {}).get('point', [0, 0])
            target_point = target_points.get(key, {}).get('point', [0, 0])

            # Get existing yaw from old config if available (or use default)
            old_config = config.get('Midan', {}).get(clm_num, {})
            old_ego_yaw = old_config.get('ego', {}).get('ego', [0, 0, 0])[2] if old_config else 0
            old_target_yaw = old_config.get('target', {}).get('ego', [0, 0, 0])[2] if old_config else 0

            # Default yaw values for each scenario (based on typical lane change directions)
            default_yaw = {
                1: 2.216,   # CLM1: right merge
                2: 2.216,   # CLM2: left merge
                3: 2.216,   # CLM3: right merge
                4: 2.216,    # CLM4: 3-lane
                5: 0.791,    # CLM5: intersection
                6: -1.102     # CLM6: intersection
            }

            ego_yaw = old_ego_yaw if old_ego_yaw != 0 else default_yaw.get(clm_num, 0)
            target_yaw = old_target_yaw if old_target_yaw != 0 else default_yaw.get(clm_num, 0)
            if clm_num == 5:
                target_yaw = -1.102 
            elif clm_num == 6:
                target_yaw = 0.791

            # Create entry
            midan_config[key] = {
                'ego': {
                    'ego': [ego_point[0], ego_point[1], ego_yaw]
                },
                'target': {
                    'ego': [target_point[0], target_point[1], target_yaw]
                },
                'obstacles': [
                    [0, 0, 0, -13, 1]
                ]
            }

            print(f"{key}:")
            print(f"  Ego: [{ego_point[0]:.2f}, {ego_point[1]:.2f}, {ego_yaw:.3f}]")
            print(f"  Target: [{target_point[0]:.2f}, {target_point[1]:.2f}, {target_yaw:.3f}]")

    # Process ETrA scenarios (1-6) - no speed variants
    for etra_num in range(1, 7):
        key = f"ETrA{etra_num}"

        # Get points (format: [x, y])
        ego_point = ego_points.get(key, {}).get('point', [0, 0])
        target_point = target_points.get(key, {}).get('point', [0, 0])

        # Default yaw values for ETrA scenarios
        default_yaw_etra = {
            1: 2.216,   # ETrA1
            2: 2.216,   # ETrA2
            3: 2.216,   # ETrA3
            4: 2.216,   # ETrA4
            5: 0.791,   # ETrA5
            6: 0.791    # ETrA6
        }

        ego_yaw = default_yaw_etra.get(etra_num, 0)
        target_yaw = default_yaw_etra.get(etra_num, 0)

        # ETrA scenarios might need different target yaw
        if etra_num == 1:
            target_yaw = 2.216
        elif etra_num == 2:
            target_yaw = 2.216
        elif etra_num == 3:
            target_yaw = 2.216
        elif etra_num == 4:
            target_yaw = 2.216
        elif etra_num == 5:
            target_yaw = 0.791
        elif etra_num == 6:
            target_yaw = 0.791

        # Create entry
        midan_config[key] = {
            'ego': {
                'ego': [ego_point[0], ego_point[1], ego_yaw]
            },
            'target': {
                'ego': [target_point[0], target_point[1], target_yaw]
            },
            'obstacles': [
                [0, 0, 0, -13, 1]
            ]
        }

        print(f"{key}:")
        print(f"  Ego: [{ego_point[0]:.2f}, {ego_point[1]:.2f}, {ego_yaw:.3f}]")
        print(f"  Target: [{target_point[0]:.2f}, {target_point[1]:.2f}, {target_yaw:.3f}]")

    # Add default (use CLM1_same)
    default_key = 'CLM1_same'
    if default_key in midan_config:
        midan_config['default'] = midan_config[default_key]

    # Keep old numbered entries (7-12) if they exist
    old_midan = config.get('Midan', {})
    for num in range(7, 13):
        if num in old_midan:
            midan_config[num] = old_midan[num]

    # Update config
    config['Midan'] = midan_config

    # Save to yaml
    with open("./yamls/ego_pose.yaml", "w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    print("\n" + "="*60)
    print("Updated ego_pose.yaml Midan section!")
    print(f"  - CLM scenarios (1-6) with speed variants: {6*3} entries")
    print(f"  - ETrA scenarios (1-6): 6 entries")
    print(f"  - Kept old numbered entries (7-12): preserved")
    print("="*60)


if __name__ == '__main__':
    update_ego_pose_yaml()
