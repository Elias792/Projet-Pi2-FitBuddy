# Workout Analyzer for Accelerometer Data

This project provides tools for analyzing workout data from accelerometer readings. It processes raw accelerometer data to detect repetitions, sets, rest periods, and measure range of motion and timing metrics.

## Overview

The Workout Analyzer is designed to work with raw accelerometer data from fitness devices or sensors. It applies a Kalman filter to reduce noise in the signal and employs peak detection algorithms to identify individual repetitions. The system can distinguish between active exercise periods (sets) and rest intervals, providing comprehensive workout metrics.

## Features

- **Signal Processing**: Applies Kalman filtering to clean noisy accelerometer data
- **Repetition Detection**: Automatically identifies individual exercise repetitions
- **Set Identification**: Distinguishes between different sets of exercises
- **Rest Period Detection**: Identifies and measures rest periods between sets
- **Range of Motion Analysis**: Calculates the relative range of motion for each repetition
- **Duration Metrics**: Measures the duration of repetitions, sets, and rest periods
- **Visualization**: Generates comprehensive multi-panel visualizations of workout data
- **Summary Statistics**: Provides detailed workout summaries including reps per set

## Usage

1. Prepare your accelerometer data in CSV format with the following columns:
   - `timestamp_ms`: Timestamps in milliseconds
   - `accX_g`, `accY_g`, `accZ_g`: Accelerometer readings in g-force units

2. Run the analyzer:
   ```python
   import numpy as np
   import matplotlib.pyplot as plt
   import pandas as pd
   from scipy.signal import find_peaks
   
   # Import the workout analyzer code
   from workout_analyzer import KalmanFilterAccelerometer, analyze_workout_from_csv, visualize_workout_analysis_from_csv
   
   # Analyze the workout
   csv_filepath = 'your_data.csv'
   results = analyze_workout_from_csv(csv_filepath, process_noise=0.003, measurement_noise=0.1)
   
   # Visualize the results
   visualize_workout_analysis_from_csv(results)
   ```

## Customization

The analyzer can be tuned for different exercises by adjusting the following parameters:

- `process_noise`: Affects how much the model trusts its predictions vs. measurements (default: 0.003)
- `measurement_noise`: Controls how much to trust sensor readings (default: 0.1)
- `threshold`: Determines what level of acceleration is considered rest (default: signal_mean * 0.4)
- `min_rest_samples`: Sets minimum duration for a rest period (default: sampling_rate * 1.0)

## Component Descriptions

### KalmanFilterAccelerometer

A class implementing the Kalman filter algorithm for accelerometer data. It reduces noise while preserving the underlying signal patterns.

### analyze_workout_from_csv

The main function that processes CSV data to extract workout metrics. It identifies repetitions, sets, and rest periods.

### visualize_workout_analysis_from_csv

Generates a multi-panel visualization showing:
1. Raw and filtered accelerometer data with detected repetitions, sets, and rest periods
2. Range of motion percentages for each repetition
3. Duration of each repetition
4. Duration of rest periods between sets

### Helper Functions

- `count_reps`: Detects repetition peaks in filtered data
- `calculate_rep_durations`: Measures the duration of each repetition
- `detect_rest_periods`: Identifies rest periods between exercise sets
- `calculate_range_of_motion`: Analyzes the movement amplitude of each repetition

## Example Output

The visualizations show:
- Acceleration patterns with detected repetition peaks
- Exercise phases color-coded by set
- Repetition range of motion compared to the maximum
- Repetition durations
- Rest periods between sets

The printed summary provides:
- Number of sets detected
- Total repetition count
- Repetitions per set
- Average and total rep duration
- Average and total rest time
- Range of motion statistics

## Requirements

- Python 3.6+
- NumPy
- Pandas
- SciPy
- Matplotlib

## Limitations

- Works best with clear, distinct repetition patterns
- May require parameter tuning for different exercise types
- Performance depends on the quality of the accelerometer data