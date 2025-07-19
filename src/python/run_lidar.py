import argparse
import pydiscocal


def main():
    parser = argparse.ArgumentParser(
        description="LiDAR camera calibration using pydiscocal"
    )
    parser.add_argument(
        "config_path",
        help="Path to the YAML configuration file"
    )
    args = parser.parse_args()



    pydiscocal.lidar_calibration(args.config_path)

if __name__ == "__main__":
    main()