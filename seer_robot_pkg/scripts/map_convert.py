#!/usr/bin/env python3

import argparse
import sys
import json

def main():
    #### ARG ###
    parser = argparse.ArgumentParser(description='Process input path and generate output')
    parser.add_argument('path', nargs='?', default='seer_robot_pkg/config/Kraiwich_Map_v.0.3.7.smap', help='Input path to process')
    parser.add_argument('-o', '--output', help='Output file path', default='seer_robot_pkg/config/map.json')
    args = parser.parse_args()
    output_path = args.output
    file_path = args.path

    ### Load data ###
    with open(file_path, 'r') as file:
        data = json.load(file)
    advancedPointList = data['advancedPointList']
    advancedCurveList = data['advancedCurveList']

    ### Load data to station name ###
    station_name = []
    for i in advancedPointList:
        station_name.append(i['instanceName'])

    ### Sort data ###
    station_name.sort(key=lambda x: int(''.join(filter(str.isdigit, x))) if any(c.isdigit() for c in x) else float('inf'))
    # print (f"Station Name: {station_name}")

    ### Auto zone with Label ###
    zone = {}
    for i in station_name:
        if 'AP' in i:
            zone[i] = {'zone': ['pallet_zone']}
        elif 'CP' in i:
            zone[i] = {'zone': ['Charger_zone']}
        else:
            zone[i] = {'zone': ['none']}
    
    try:
        # Process and output JSON data
        output_data = {
            "advancedPointList": advancedPointList,
            "advancedCurveList": advancedCurveList,
            "zone": zone
        }
        
        with open(output_path, 'w') as f:
            json.dump(output_data, f, indent=2)
        
        print(f"JSON output saved to: {output_path}")
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()