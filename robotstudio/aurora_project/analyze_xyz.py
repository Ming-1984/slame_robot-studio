#!/usr/bin/env python3
import sys

def analyze_xyz_file(filename):
    """Analyze XYZ file to understand coordinate ranges and potential issues"""
    
    points = []
    colors = []
    
    try:
        with open(filename, 'r') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if not line:
                    continue
                    
                parts = line.split()
                if len(parts) != 6:
                    print(f"Warning: Line {line_num} has {len(parts)} parts instead of 6: {line}")
                    continue
                
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                    
                    points.append((x, y, z))
                    colors.append((r, g, b))
                    
                except ValueError as e:
                    print(f"Warning: Line {line_num} has invalid values: {line} - {e}")
                    continue
    
    except FileNotFoundError:
        print(f"Error: File {filename} not found")
        return
    
    if not points:
        print("Error: No valid points found in file")
        return
    
    # Calculate ranges
    xs, ys, zs = zip(*points)
    
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)
    
    print(f"=== XYZ File Analysis: {filename} ===")
    print(f"Total points: {len(points)}")
    print(f"")
    print(f"Coordinate Ranges:")
    print(f"  X: {min_x:.6f} to {max_x:.6f} (range: {max_x - min_x:.6f})")
    print(f"  Y: {min_y:.6f} to {max_y:.6f} (range: {max_y - min_y:.6f})")
    print(f"  Z: {min_z:.6f} to {max_z:.6f} (range: {max_z - min_z:.6f})")
    print(f"")
    
    # Check for potential issues
    issues = []
    
    # Check for very large coordinate values
    max_coord = max(abs(min_x), abs(max_x), abs(min_y), abs(max_y), abs(min_z), abs(max_z))
    if max_coord > 1000:
        issues.append(f"Very large coordinates detected (max: {max_coord:.2f})")
    
    # Check for very small ranges
    x_range = max_x - min_x
    y_range = max_y - min_y
    z_range = max_z - min_z
    
    if x_range < 0.001:
        issues.append(f"Very small X range: {x_range:.6f}")
    if y_range < 0.001:
        issues.append(f"Very small Y range: {y_range:.6f}")
    if z_range < 0.001:
        issues.append(f"Very small Z range: {z_range:.6f}")
    
    # Check color ranges
    rs, gs, bs = zip(*colors)
    min_r, max_r = min(rs), max(rs)
    min_g, max_g = min(gs), max(gs)
    min_b, max_b = min(bs), max(bs)
    
    print(f"Color Ranges:")
    print(f"  R: {min_r} to {max_r}")
    print(f"  G: {min_g} to {max_g}")
    print(f"  B: {min_b} to {max_b}")
    print(f"")
    
    # Check for color issues
    if max_r > 255 or max_g > 255 or max_b > 255:
        issues.append("Color values exceed 255")
    if min_r < 0 or min_g < 0 or min_b < 0:
        issues.append("Negative color values detected")
    
    # Sample points
    print(f"Sample Points:")
    for i in [0, len(points)//4, len(points)//2, 3*len(points)//4, len(points)-1]:
        x, y, z = points[i]
        r, g, b = colors[i]
        print(f"  Point {i+1}: ({x:.6f}, {y:.6f}, {z:.6f}) RGB({r}, {g}, {b})")
    print(f"")
    
    # Report issues
    if issues:
        print(f"POTENTIAL ISSUES DETECTED:")
        for issue in issues:
            print(f"  - {issue}")
        print(f"")
        
        # Suggest fixes
        print(f"SUGGESTED FIXES:")
        if max_coord > 1000:
            print(f"  - Coordinates seem very large. CloudCompare might have trouble with scale.")
            print(f"  - Consider scaling down coordinates by factor of 100 or 1000")
        
        if any("very small" in issue.lower() for issue in issues):
            print(f"  - Some coordinate ranges are very small")
            print(f"  - Points might appear clustered or invisible in CloudCompare")
    else:
        print(f"No obvious issues detected with the XYZ format.")
    
    print(f"")
    print(f"CloudCompare Import Tips:")
    print(f"  1. Make sure to select 'ASCII' format when importing")
    print(f"  2. Specify column order: X Y Z R G B")
    print(f"  3. Check 'Colors' checkbox in import dialog")
    print(f"  4. If points don't appear, try 'Fit in Window' or check coordinate scale")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python analyze_xyz.py <xyz_file>")
        sys.exit(1)
    
    analyze_xyz_file(sys.argv[1])
