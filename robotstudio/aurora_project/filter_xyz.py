#!/usr/bin/env python3
"""
Filter XYZ point cloud file to remove outliers with abnormally large coordinates
"""
import sys
import os

def filter_xyz_file(input_file, output_file, max_coordinate=50.0):
    """
    Filter XYZ file to remove points with coordinates larger than max_coordinate
    
    Args:
        input_file: Input XYZ file path
        output_file: Output XYZ file path  
        max_coordinate: Maximum allowed coordinate value (default: 50 meters)
    """
    
    if not os.path.exists(input_file):
        print(f"Error: Input file {input_file} not found")
        return False
    
    points_read = 0
    points_written = 0
    points_filtered = 0
    
    try:
        with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
            for line_num, line in enumerate(infile, 1):
                line = line.strip()
                if not line:
                    continue
                
                points_read += 1
                parts = line.split()
                
                if len(parts) != 6:
                    print(f"Warning: Line {line_num} has {len(parts)} parts instead of 6, skipping")
                    continue
                
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                    
                    # Check if any coordinate is too large
                    max_coord = max(abs(x), abs(y), abs(z))
                    
                    if max_coord > max_coordinate:
                        points_filtered += 1
                        if points_filtered <= 10:  # Show first 10 filtered points
                            print(f"Filtered point {points_filtered}: ({x:.6f}, {y:.6f}, {z:.6f}) - max coord: {max_coord:.6f}")
                        continue
                    
                    # Write valid point
                    outfile.write(f"{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n")
                    points_written += 1
                    
                except ValueError as e:
                    print(f"Warning: Line {line_num} has invalid values: {line} - {e}")
                    continue
    
    except Exception as e:
        print(f"Error processing file: {e}")
        return False
    
    print(f"\nFiltering Results:")
    print(f"  Points read: {points_read}")
    print(f"  Points written: {points_written}")
    print(f"  Points filtered out: {points_filtered}")
    print(f"  Filter threshold: {max_coordinate} meters")
    
    if points_filtered > 10:
        print(f"  (Only showed first 10 filtered points)")
    
    # Calculate new coordinate ranges
    if points_written > 0:
        print(f"\nAnalyzing filtered file...")
        analyze_filtered_file(output_file)
    
    return True

def analyze_filtered_file(filename):
    """Analyze the filtered file to show new coordinate ranges"""
    
    points = []
    
    try:
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) >= 3:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    points.append((x, y, z))
    except Exception as e:
        print(f"Error analyzing filtered file: {e}")
        return
    
    if not points:
        print("No points in filtered file")
        return
    
    xs, ys, zs = zip(*points)
    
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)
    
    print(f"Filtered Coordinate Ranges:")
    print(f"  X: {min_x:.6f} to {max_x:.6f} (range: {max_x - min_x:.6f})")
    print(f"  Y: {min_y:.6f} to {max_y:.6f} (range: {max_y - min_y:.6f})")
    print(f"  Z: {min_z:.6f} to {max_z:.6f} (range: {max_z - min_z:.6f})")
    
    # Check if ranges are reasonable for CloudCompare
    max_range = max(max_x - min_x, max_y - min_y, max_z - min_z)
    if max_range < 100:
        print(f"✓ Coordinate ranges look reasonable for CloudCompare import")
    else:
        print(f"⚠ Coordinate ranges are still quite large, may need further filtering")

def main():
    if len(sys.argv) < 3:
        print("Usage: python filter_xyz.py <input_file> <output_file> [max_coordinate]")
        print("  input_file: Input XYZ file to filter")
        print("  output_file: Output filtered XYZ file")
        print("  max_coordinate: Maximum allowed coordinate value in meters (default: 50)")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    max_coordinate = float(sys.argv[3]) if len(sys.argv) > 3 else 50.0
    
    print(f"Filtering XYZ file: {input_file}")
    print(f"Output file: {output_file}")
    print(f"Maximum coordinate threshold: {max_coordinate} meters")
    print(f"=" * 50)
    
    success = filter_xyz_file(input_file, output_file, max_coordinate)
    
    if success:
        print(f"\n✓ Filtering completed successfully!")
        print(f"✓ Filtered file saved as: {output_file}")
        print(f"\nCloudCompare Import Tips:")
        print(f"  1. Import the filtered file: {output_file}")
        print(f"  2. Select 'ASCII' format")
        print(f"  3. Column order: X Y Z R G B")
        print(f"  4. Check 'Colors' checkbox")
        print(f"  5. Use 'Fit in Window' to see all points")
    else:
        print(f"\n✗ Filtering failed!")
        sys.exit(1)

if __name__ == "__main__":
    main()
