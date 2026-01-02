# ROS Basic Setup

This repository provides a basic **ROS 2 workspace (`ros2_ws`)** setup with **CI/CD integration** using **GitHub Actions**, including **linters for Python and C/C++**.

---

## 📁 Repository Structure

```bash
.
├── README.md
├── requirements.txt         # Python dependencies
├── ros2_ws                  # ROS 2 workspace
├── ruff.toml                # Ruff configuration 
├── scripts
│   └── build-ros.sh         # Script to build the ROS2 ws
└── .github/workflows        # GitHub Actions workflows
    ├── build.yaml           # ROS 2 ws build workflow
    ├── analysis-python.yaml # Python linter workflow 
    └── analysis-cpp.yaml    # C/C++ linter
```

## ⚙️ GitHub Actions / CI

This repository is configured with **GitHub Actions** to automate the build, code validation, and style checks:

### 1. `build.yaml`
- Builds the ROS 2 workspace (`ros2_ws`) using `colcon build`.
- Runs inside an official `ros:humble-ros-base` container.
- Prepares the workspace for testing or deployment.
- Triggered on **push or pull request to `main`**.

### 2. `analysis-python.yaml`
- Runs **Ruff** to check Python code style and quality.
- Runs **MyPy** for static type checking.
- Configuration is in `ruff.toml` and `.mypy.ini`.
- Triggered only when `*.py` files are modified.

### 3. `analysis-cpp.yaml`
- Runs **clang-tidy** for static analysis of C/C++ code.
- Runs **clang-format** to check code formatting.
- Linter configuration is in `.clang-tidy` and `.clang-format`.
- Runs on all C/C++ files in the workspace (`src/`).

---

## 🛠️ Scripts

### `scripts/build-ros.sh`
- Script to build the ROS 2 workspace.
- Supports clean builds or custom CMake arguments.
- Used by the `build.yaml` workflow.

```bash
# Example usage locally
./scripts/build-ros.sh
# For a clean build
./scripts/build-ros.sh --clean
``` 

## 🚀 Usage

### Clone and set up

```bash
git clone <repo-url>
cd ros_basic_setup
# Install Python dependencies
pip install -r requirements.txt
# Build the ROS 2 workspace
./scripts/build-ros.sh
``` 

## 🧹 Using the Linters

### Python
- **Ruff**: checks code style and quality.
- **MyPy**: static type checking.

```bash
# Run manually
ruff check $(git ls-files '*.py')
mypy $(git ls-files '*.py') --config-file .mypy.ini
``` 

### C/C++
- **clang-tidy**: static analysis.
- **clang-format**: code formatting.
```bash
# Run manually
cd ros2_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
clang-tidy -p build $(git ls-files '*.cpp' '*.h', '*.hpp')
clang-format -i $(git ls-files '*.cpp' '*.h', '*.hpp')
``` 