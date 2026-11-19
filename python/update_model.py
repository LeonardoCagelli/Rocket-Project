from pathlib import Path
import shutil
import sys

# Project root directory
PROJECT_ROOT = Path(__file__).resolve().parent.parent

# Source and destination model paths
SOURCE_MODEL = PROJECT_ROOT / "models" / "dqn_rocket_actor.onnx"

DESTINATION_MODEL = (
    PROJECT_ROOT
    / "build-rl"
    / "BalloonInterceptor.app"
    / "Contents"
    / "MacOS"
    / "dqn_rocket_actor.onnx"
)


def main():
    # Check that the source model exists
    if not SOURCE_MODEL.is_file():
        print(f"Error: Source model not found:\n{SOURCE_MODEL}")
        sys.exit(1)

    # Check that the application directory exists
    if not DESTINATION_MODEL.parent.is_dir():
        print(
            "Error: Application directory not found:\n"
            f"{DESTINATION_MODEL.parent}"
        )
        sys.exit(1)

    # Copy and replace the old model
    shutil.copy2(SOURCE_MODEL, DESTINATION_MODEL)

    # Verify that the copied files have the same size
    if SOURCE_MODEL.stat().st_size != DESTINATION_MODEL.stat().st_size:
        print("Error: The copied model size does not match.")
        sys.exit(1)

    print("DQN model updated successfully!")
    print(f"Source:      {SOURCE_MODEL}")
    print(f"Destination: {DESTINATION_MODEL}")
    print(f"Model size:  {DESTINATION_MODEL.stat().st_size:,} bytes")


if __name__ == "__main__":
    main()