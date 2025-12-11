#!/usr/bin/env python3
"""
ArUco Marker Generator

Generates ArUco markers for printing and deployment.
Saves markers as PNG files that can be printed.

Usage:
    python3 generate_markers.py --ids 0 1 2 3 --size 200
    python3 generate_markers.py --all --dict 4X4_50

Requirements:
    pip install opencv-contrib-python numpy
"""

import cv2
import numpy as np
import argparse
import os


# Available ArUco dictionaries
ARUCO_DICT = {
    "4X4_50": cv2.aruco.DICT_4X4_50,
    "4X4_100": cv2.aruco.DICT_4X4_100,
    "4X4_250": cv2.aruco.DICT_4X4_250,
    "4X4_1000": cv2.aruco.DICT_4X4_1000,
    "5X5_50": cv2.aruco.DICT_5X5_50,
    "5X5_100": cv2.aruco.DICT_5X5_100,
    "5X5_250": cv2.aruco.DICT_5X5_250,
    "5X5_1000": cv2.aruco.DICT_5X5_1000,
    "6X6_50": cv2.aruco.DICT_6X6_50,
    "6X6_100": cv2.aruco.DICT_6X6_100,
    "6X6_250": cv2.aruco.DICT_6X6_250,
    "6X6_1000": cv2.aruco.DICT_6X6_1000,
    "7X7_50": cv2.aruco.DICT_7X7_50,
    "7X7_100": cv2.aruco.DICT_7X7_100,
    "7X7_250": cv2.aruco.DICT_7X7_250,
    "7X7_1000": cv2.aruco.DICT_7X7_1000,
}


def generate_marker(marker_id, dict_name, size_pixels, border_bits=1):
    """
    Generate an ArUco marker image.
    
    Args:
        marker_id: ID of the marker (must be within dictionary range)
        dict_name: Name of ArUco dictionary (e.g., "4X4_50")
        size_pixels: Size of marker in pixels
        border_bits: Width of white border in bits
    
    Returns:
        numpy array: Marker image
    """
    if dict_name not in ARUCO_DICT:
        raise ValueError(f"Unknown dictionary: {dict_name}")
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dict_name])
    
    # Generate marker
    marker_image = cv2.aruco.generateImageMarker(
        aruco_dict, marker_id, size_pixels, borderBits=border_bits
    )
    
    return marker_image


def save_marker(marker_image, marker_id, output_dir, dict_name):
    """Save marker to file."""
    os.makedirs(output_dir, exist_ok=True)
    
    filename = f"marker_{dict_name}_{marker_id}.png"
    filepath = os.path.join(output_dir, filename)
    
    cv2.imwrite(filepath, marker_image)
    return filepath


def create_marker_sheet(marker_ids, dict_name, marker_size, output_dir):
    """
    Create a sheet with multiple markers for printing.
    
    Args:
        marker_ids: List of marker IDs
        dict_name: ArUco dictionary name
        marker_size: Size of each marker in pixels
        output_dir: Output directory
    """
    # Arrange markers in a grid
    cols = min(4, len(marker_ids))
    rows = (len(marker_ids) + cols - 1) // cols
    
    spacing = 50  # pixels between markers
    label_height = 60  # pixels for label text
    
    sheet_width = cols * (marker_size + spacing) + spacing
    sheet_height = rows * (marker_size + spacing + label_height) + spacing
    
    # Create white sheet
    sheet = np.ones((sheet_height, sheet_width), dtype=np.uint8) * 255
    
    for idx, marker_id in enumerate(marker_ids):
        row = idx // cols
        col = idx % cols
        
        # Generate marker
        marker = generate_marker(marker_id, dict_name, marker_size)
        
        # Position on sheet
        x = spacing + col * (marker_size + spacing)
        y = spacing + row * (marker_size + spacing + label_height)
        
        # Place marker
        sheet[y:y+marker_size, x:x+marker_size] = marker
        
        # Add label
        label = f"ID: {marker_id}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.2
        thickness = 2
        text_size = cv2.getTextSize(label, font, font_scale, thickness)[0]
        text_x = x + (marker_size - text_size[0]) // 2
        text_y = y + marker_size + 40
        cv2.putText(sheet, label, (text_x, text_y), font, font_scale, 0, thickness)
    
    # Save sheet
    filename = f"marker_sheet_{dict_name}.png"
    filepath = os.path.join(output_dir, filename)
    cv2.imwrite(filepath, sheet)
    
    return filepath


def main():
    parser = argparse.ArgumentParser(
        description="Generate ArUco markers for robot localization",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate markers 0, 1, 2, 3 from default dictionary
  python3 generate_markers.py --ids 0 1 2 3
  
  # Generate all markers for your project (0-3)
  python3 generate_markers.py --all
  
  # Generate with specific dictionary and size
  python3 generate_markers.py --ids 0 1 2 3 --dict 5X5_100 --size 300
  
  # Create a single sheet with all markers
  python3 generate_markers.py --all --sheet --size 400

Printing Tips:
  - Print at actual size (no scaling)
  - Use white paper, preferably matte
  - Mount on rigid backing (cardboard, foam board)
  - Recommended size: 10-20cm for 1-3 meter detection
  - Ensure good contrast and no glare
        """
    )
    
    parser.add_argument(
        '--ids', type=int, nargs='+',
        help='Marker IDs to generate (e.g., --ids 0 1 2 3)'
    )
    parser.add_argument(
        '--all', action='store_true',
        help='Generate markers 0-3 (default set)'
    )
    parser.add_argument(
        '--dict', type=str, default='4X4_50',
        choices=ARUCO_DICT.keys(),
        help='ArUco dictionary to use (default: 4X4_50)'
    )
    parser.add_argument(
        '--size', type=int, default=200,
        help='Marker size in pixels (default: 200)'
    )
    parser.add_argument(
        '--output', type=str, default='aruco_markers',
        help='Output directory (default: aruco_markers)'
    )
    parser.add_argument(
        '--sheet', action='store_true',
        help='Generate a single sheet with all markers'
    )
    
    args = parser.parse_args()
    
    # Determine which markers to generate
    if args.all:
        marker_ids = [0, 1, 2, 3]
    elif args.ids:
        marker_ids = args.ids
    else:
        print("Error: Specify either --ids or --all")
        parser.print_help()
        return
    
    print("=" * 60)
    print("ArUco Marker Generator")
    print("=" * 60)
    print(f"Dictionary: {args.dict}")
    print(f"Marker IDs: {marker_ids}")
    print(f"Size: {args.size} pixels")
    print(f"Output: {args.output}/")
    print("=" * 60)
    
    # Generate individual markers
    print("\nGenerating individual markers...")
    for marker_id in marker_ids:
        marker = generate_marker(marker_id, args.dict, args.size)
        filepath = save_marker(marker, marker_id, args.output, args.dict)
        print(f"  ✓ Generated: {filepath}")
    
    # Generate sheet if requested
    if args.sheet:
        print("\nGenerating marker sheet...")
        sheet_path = create_marker_sheet(marker_ids, args.dict, args.size, args.output)
        print(f"  ✓ Generated: {sheet_path}")
    
    print("\n" + "=" * 60)
    print("✓ Complete!")
    print("=" * 60)
    print("\nNext steps:")
    print("1. Print markers at actual size (disable scaling)")
    print("2. Mount on rigid surface")
    print("3. Measure and record positions in marker_map.yaml")
    print("4. Place in environment with good lighting")
    print("\nRecommended marker size for printing:")
    print("  - 10cm x 10cm for detection up to 1.5m")
    print("  - 15cm x 15cm for detection up to 2.5m")
    print("  - 20cm x 20cm for detection up to 3.5m")


if __name__ == '__main__':
    main()
