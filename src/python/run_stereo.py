import argparse
import pydiscocal

def main():
    parser = argparse.ArgumentParser(
        description="Stereo calibration using pydiscocal"
    )
    parser.add_argument(
        "config_path",
        help="Path to the YAML configuration file"
    )
    args = parser.parse_args()

    # 실제 calibration 호출
    pydiscocal.stereo_calibration(args.config_path)

if __name__ == "__main__":
    main()
