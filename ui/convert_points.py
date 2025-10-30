#!/usr/bin/env python3
"""
Convert GPS coordinates (LLA) from points.txt to ENU coordinates
and save to ego_point.yaml and target_point.yaml
"""

import math
import yaml


def lla_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """
    Convert LLA (Latitude, Longitude, Altitude) to ENU (East, North, Up)

    Args:
        lat, lon, alt: Target point coordinates (degrees, degrees, meters)
        lat0, lon0, alt0: Reference point coordinates (degrees, degrees, meters)

    Returns:
        (east, north, up): ENU coordinates in meters
    """
    # Convert degrees to radians
    lat = math.radians(lat)
    lon = math.radians(lon)
    lat0 = math.radians(lat0)
    lon0 = math.radians(lon0)

    # WGS84 ellipsoid parameters
    a = 6378137.0  # semi-major axis (meters)
    f = 1 / 298.257223563  # flattening
    e2 = 2 * f - f * f  # first eccentricity squared

    # Calculate N (radius of curvature in prime vertical)
    N0 = a / math.sqrt(1 - e2 * math.sin(lat0) ** 2)
    N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)

    # Convert LLA to ECEF (Earth-Centered, Earth-Fixed)
    x0 = (N0 + alt0) * math.cos(lat0) * math.cos(lon0)
    y0 = (N0 + alt0) * math.cos(lat0) * math.sin(lon0)
    z0 = (N0 * (1 - e2) + alt0) * math.sin(lat0)

    x = (N + alt) * math.cos(lat) * math.cos(lon)
    y = (N + alt) * math.cos(lat) * math.sin(lon)
    z = (N * (1 - e2) + alt) * math.sin(lat)

    # ECEF difference
    dx = x - x0
    dy = y - y0
    dz = z - z0

    # Convert ECEF difference to ENU
    east = -math.sin(lon0) * dx + math.cos(lon0) * dy
    north = -math.sin(lat0) * math.cos(lon0) * dx - math.sin(lat0) * math.sin(lon0) * dy + math.cos(lat0) * dz
    up = math.cos(lat0) * math.cos(lon0) * dx + math.cos(lat0) * math.sin(lon0) * dy + math.sin(lat0) * dz

    return east, north, up


def parse_points_txt(filename):
    """
    Parse points.txt file

    Returns:
        dict: {
            'CLM1': {'ego': [(lat1, lon1), (lat2, lon2), (lat3, lon3)],
                     'target': [(lat1, lon1), (lat2, lon2), (lat3, lon3)]},
            'CLM2': {...},
            'ETrA1': {'ego': [(lat1, lon1)], 'target': [(lat1, lon1)]},
            ...
        }
    """
    data = {}
    current_scenario = None
    current_type = None  # 'ego' or 'target'

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            if line.startswith('CLM') or line.startswith('ETrA'):
                current_scenario = line
                data[current_scenario] = {'ego': [], 'target': []}
            elif line.lower() == 'ego':
                current_type = 'ego'
            elif line.lower() == 'target':
                current_type = 'target'
            else:
                # Parse coordinate line
                if current_scenario and current_type:
                    parts = line.split(',')
                    if len(parts) == 2:
                        lat = float(parts[0])
                        lon = float(parts[1])
                        data[current_scenario][current_type].append((lat, lon))

    return data


def convert_and_save():
    """Main conversion function"""
    # Base LLA coordinates
    base_lat = 37.527247
    base_lon = 126.506810
    base_alt = 7.0

    # Parse points.txt
    points_data = parse_points_txt('./yamls/points.txt')

    # Prepare dictionaries for YAML output
    ego_points = {}
    target_points = {}

    # Speed variant names (for CLM scenarios only)
    speed_variants = ['slower', 'same', 'faster']

    # Convert each scenario
    for scenario_name, scenario_data in sorted(points_data.items()):
        print(f"\n{scenario_name}:")

        # Check if this is ETrA scenario (no speed variants)
        is_etra = scenario_name.startswith('ETrA')

        if is_etra:
            # ETrA scenarios: single point, no speed variants
            if scenario_data['ego']:
                lat, lon = scenario_data['ego'][0]
                east, north, up = lla_to_enu(lat, lon, base_alt, base_lat, base_lon, base_alt)
                ego_points[scenario_name] = {'point': [east, north]}
                print(f"  Ego: ({lat:.8f}, {lon:.8f}) -> ({east:.2f}, {north:.2f})")

            if scenario_data['target']:
                lat, lon = scenario_data['target'][0]
                east, north, up = lla_to_enu(lat, lon, base_alt, base_lat, base_lon, base_alt)
                target_points[scenario_name] = {'point': [east, north]}
                print(f"  Target: ({lat:.8f}, {lon:.8f}) -> ({east:.2f}, {north:.2f})")

        else:
            # CLM scenarios: three points with speed variants
            # Process ego points
            for idx, (lat, lon) in enumerate(scenario_data['ego']):
                if idx >= 3:
                    break

                east, north, up = lla_to_enu(lat, lon, base_alt, base_lat, base_lon, base_alt)
                key = f"{scenario_name}_{speed_variants[idx]}"
                ego_points[key] = {'point': [east, north]}

                print(f"  Ego {speed_variants[idx]}: ({lat:.8f}, {lon:.8f}) -> ({east:.2f}, {north:.2f})")

            # Process target points
            for idx, (lat, lon) in enumerate(scenario_data['target']):
                if idx >= 3:
                    break

                east, north, up = lla_to_enu(lat, lon, base_alt, base_lat, base_lon, base_alt)
                key = f"{scenario_name}_{speed_variants[idx]}"
                target_points[key] = {'point': [east, north]}

                print(f"  Target {speed_variants[idx]}: ({lat:.8f}, {lon:.8f}) -> ({east:.2f}, {north:.2f})")

    # Add default points (use CLM1 same as default)
    if 'CLM1_same' in ego_points:
        ego_points['default'] = ego_points['CLM1_same']
    if 'CLM1_same' in target_points:
        target_points['default'] = target_points['CLM1_same']

    # Save to YAML files
    with open('./yamls/ego_point.yaml', 'w') as f:
        yaml.dump(ego_points, f, default_flow_style=False, sort_keys=False)

    with open('./yamls/target_point.yaml', 'w') as f:
        yaml.dump(target_points, f, default_flow_style=False, sort_keys=False)

    print("\n" + "="*60)
    print("Conversion complete!")
    print(f"Saved {len(ego_points)-1} ego points to ego_point.yaml")
    print(f"Saved {len(target_points)-1} target points to target_point.yaml")
    print("="*60)


if __name__ == '__main__':
    convert_and_save()


37.5283792,126.5209355