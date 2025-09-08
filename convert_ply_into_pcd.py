import open3d as o3d
import os
import argparse

def convert_ply_to_pcd(input_dir, output_dir):
    os.makedirs(output_dir, exist_ok=True)

    ply_files = [f for f in os.listdir(input_dir) if f.endswith(".ply")]
    for filename in ply_files:
        input_path = os.path.join(input_dir, filename)
        output_path = os.path.join(output_dir, filename.replace(".ply", ".pcd"))

        try:
            pcd = o3d.io.read_point_cloud(input_path)
            print(f"[INFO] Loaded: {filename} ({len(pcd.points)} points)")

            o3d.io.write_point_cloud(output_path, pcd)
            print(f"[INFO] Saved as PCD to: {output_path}")
        except Exception as e:
            print(f"[ERROR] Failed to convert {filename}: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert .ply files to .pcd format")
    parser.add_argument("input_dir", help="Input directory containing .ply files")
    parser.add_argument("output_dir", help="Output directory to save .pcd files")
    args = parser.parse_args()

    convert_ply_to_pcd(args.input_dir, args.output_dir)
