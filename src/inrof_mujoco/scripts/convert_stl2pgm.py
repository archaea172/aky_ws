import numpy as np
import trimesh
from PIL import Image
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
import argparse
import math
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description='Convert STL file to PGM image')
    parser.add_argument('--stl', required=True, help='Path to the input STL file')
    parser.add_argument('--output', required=True, help='Path to the output PGM file directory')
    parser.add_argument('--resolution', type=float, default=0.01, help='Resolution of the output image')
    parser.add_argument('--z_min', type=float, default=0.1, help='Minimum Z value to consider for occupancy')
    parser.add_argument('--z_max', type=float, default=1.0, help='Maximum Z value to consider for occupancy')

    args = parser.parse_args()

    stl_path = args.stl
    output_dir = Path(args.output)
    resolution = args.resolution
    z_min = args.z_min
    z_max = args.z_max

    mesh = trimesh.load(stl_path)
    
    polys = []
    for tri in mesh.triangles:
        if tri[:, 2].max() < z_min or tri[:, 2].min() > z_max:
            continue

        poly = Polygon(tri[:, :2])
        if poly.is_valid and poly.area > 0:
            polys.append(poly)

    occupied_area = unary_union(polys)

    min_x, min_y, _ = mesh.bounds[0]
    max_x, max_y, _ = mesh.bounds[1]

    width = math.ceil((max_x - min_x) / resolution)
    height = math.ceil((max_y - min_y) / resolution)

    img = np.full((height, width), 255, dtype=np.uint8)

    for iy in range(height):
        for ix in range(width):
            x = min_x + (ix + 0.5) * resolution
            y = min_y + (iy + 0.5) * resolution
            if occupied_area.covers(Point(x, y)):
                img[height - 1 - iy, ix] = 0

    pgm_path = output_dir / f"{Path(stl_path).stem}.pgm"
    Image.fromarray(img, mode='L').save(pgm_path)

    print(f"PGM image saved to {pgm_path}")

    yaml_path = output_dir / f"{Path(stl_path).stem}.yaml"

    yaml_text = (
        f"image: {Path(pgm_path).name}\n"
        "mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{min_x}, {min_y}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )

    yaml_path.write_text(yaml_text)
    print(f"YAML file saved to {yaml_path}")

if __name__ == "__main__":
    main()