# Abaqus Macros

This repository contains a collection of Python and MATLAB scripts designed to automate and enhance simulations in Abaqus, a leading Finite Element Analysis (FEA) software. These macros facilitate various tasks, including composite material analysis, mesh convergence studies, and Design of Experiments (DOE) for Single Lap Joints (SLJ).

## Contents

The repository is organized into the following directories:

1. **Composite Pin-Loaded Strap**:
   - **Description**: This directory contains scripts for simulating composite straps subjected to pin loading. The macros automate the setup and analysis processes, enabling efficient evaluation of stress distributions and failure modes in composite materials.

2. **Isight Mesh Convergence Framework/SLJ DOE**:
   - **Description**: This directory includes macros that integrate with the Isight framework to perform mesh convergence studies and DOE analyses on Single Lap Joints. The scripts help in assessing the impact of various parameters on the performance and reliability of SLJs.

3. **Pin-Loaded Composite**:
   - **Description**: This directory offers macros focused on the analysis of pin-loaded composite structures. The scripts streamline the modeling and simulation processes, facilitating the investigation of mechanical behavior under different loading conditions.

## Prerequisites

Before using these macros, ensure the following:

- **Abaqus**: Installed and licensed on your system.
- **Python**: The macros are primarily written in Python; ensure compatibility with the Python version supported by your Abaqus installation.
- **MATLAB**: Required for executing any MATLAB scripts included in the repository.
- **Isight**: Necessary for macros that integrate with the Isight framework for advanced simulation workflows.

## Usage

To utilize these macros in your Abaqus simulations:

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/semajrd1/Abaqus-Macros.git
   cd Abaqus-Macros
   ```

2. **Navigate to the Desired Directory**:
   - Choose the directory relevant to your analysis (e.g., `Composite Pin-Loaded Strap`).

3. **Load and Execute the Macro in Abaqus**:
   - Open Abaqus/CAE.
   - Navigate to the appropriate module (e.g., Part, Assembly).
   - Access the Macro Manager via `File` > `Macro Manager`.
   - Create a new macro or run an existing one by selecting the script from the repository.

4. **Follow Script-Specific Instructions**:
   - Each macro may have unique requirements or steps. Refer to any accompanying documentation or comments within the scripts for detailed guidance.

## Contributions

Contributions to enhance and expand this collection of macros are welcome. Please fork the repository, make your improvements, and submit a pull request for review.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for detailed terms and conditions.
