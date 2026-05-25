[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/hO5AKokC)
# Graph SLAM

## Introduction

In this assignment you will implement graph SLAM for a moving robot using GTSAM, a library aimed for implementations of mapping approaches in robotics. The assignment will start with a walk-through of the set-up of the graph and the corresponding measurements. After that, you will be extending this graph based on the next movements and measurements. 

Each function is implemented in a separate file in `src/`. The notebook walks you through the implementations and tell you which file to edit at each step. After implementing a function, you run the corresponding test to verify your implementation is correct.

## Setup


`gtsam` is provided through conda-forge on Windows. `requirements.txt` cannot install Conda/Miniconda, so install one of these first:

- Miniconda: https://docs.conda.io/en/latest/miniconda.html
- Anaconda: https://www.anaconda.com/download

Select your operating system (Windows/Mac/Linux) and download the installer (for linux, choose "64-Bit (x86) Installer"). Follow the installation instructions. After installation, open a new terminal and verify:

```bash
conda --version
```

On Windows, this may not work right away in VSCode. If you get an error "conda : The term 'conda' is not recognized as the name of a cmdlet, function, script file, or operable program.", press:

```bash
Ctrl + Shift + P
```

Then, run:

```bash
Python: Select Interpreter
```

And select:

```bash
('base') Python 3.x (Conda)
```

Or something similar. Your terminal should now show (base) at the start of the line and the command should work:

```bash
conda --version
```



Now, install GTSAM via conda-forge:

```bash
conda env create -f environment.yml
conda activate graph-slam
```


## Project Structure

```
graph-slam-assigment/
├── src/                   # Your implementations go here
├── tests/                 # Do not modify these
├── assignment.ipynb
├── environment.yml
├── README.md
└── requirements.txt

```

## Workflow

1. Open `assignment.ipynb` and read the explanation for a task
2. Edit the corresponding file in `src/`
3. Run the test cell in the notebook to check your implementation
4. Move on to the next task

You can also run all tests from the terminal with `pytest`.

## Important Notes

- Do not alter any code besides what is instructed by the #TODO's.
- Run the notebooks from the project root directory, otherwise the imports from `src/` and the `!pytest` calls will not work. In VSCode, open the folder `graph-slam-assignment/` directly rather than a subfolder.
- Do not modify any files in `tests/`.