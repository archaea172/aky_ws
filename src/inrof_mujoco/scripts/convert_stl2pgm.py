import numpy as np
import trimesh
from PIL import Image
from shapely.geometry import Polygon, box
from shapely.ops import unary_union
import argparse
import math

def main():
    parser = argparse.ArgumentParser(description='Convert STL file to PGM image')
    parser.add_argument('--stl', required=True, help='Path to the input STL file')
    parser.add_argument('--pgm', required=True, help='Path to the output PGM file')
    parser.add_argument('--resolution', type=float, default=0.05, help='Resolution of the output image')
    parser.add_argument('--z_min', type=float, default=0.1, help='Minimum Z value to consider for occupancy')
    parser.add_argument('--z_max', type=float, default=1.0, help='Maximum Z value to consider for occupancy')

    args = parser.parse_args()

    stl_path = args.stl
    pgm_path = args.pgm
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
            x_0 = min_x + ix * resolution
            y_0 = min_y + iy * resolution
            cell = box(x_0, y_0, x_0 + resolution, y_0 + resolution)
            if occupied_area.intersects(cell):
                img[height - 1 - iy, ix] = 0

    Image.fromarray(img, mode='L').save(pgm_path)

    print(f"PGM image saved to {pgm_path}")

if __name__ == "__main__":
    main()