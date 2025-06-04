import argparse
import pydiscocal
import yaml
from evaluation import evaluate

def main():
    parser = argparse.ArgumentParser(
        description="Mono calibration using pydiscocal"
    )
    parser.add_argument(
        "config_path",
        help="Path to the YAML configuration file"
    )
    args = parser.parse_args()

    # 실제 calibration 호출
    pydiscocal.mono_calibration(args.config_path)
    print("Start evaluation ...")

    with open(args.config_path, 'r') as f:
        cal_params = yaml.full_load(f)

    if cal_params["options"]["evaluation"]:
        img_dir = cal_params["camera"]["img_dir"]
        if img_dir[-1] != '/':
            img_dir += '/'

        results_dir = img_dir+"calibration_results/"

        with open(results_dir+"intrinsic_parameters.yaml", 'r') as f:
            params = yaml.full_load(f)

        evaluate(params)

if __name__ == "__main__":
    main()