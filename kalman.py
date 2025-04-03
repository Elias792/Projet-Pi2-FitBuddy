import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks


class KalmanFilterAccelerometer:
    """
    Kalman filter implementation for processing accelerometer data.
    
    This class provides methods to filter noisy accelerometer data and
    extract meaningful workout metrics.
    """
    
    def __init__(self, process_noise=0.01, measurement_noise=0.1, dimension=3):
        """
        Initialize the Kalman filter with appropriate parameters.
        
        Parameters:
        -----------
        process_noise : float
            The process noise covariance (Q) representing uncertainty in the model.
            Lower values mean we trust our model more than measurements.
            For workout data, this is typically low (0.001-0.05).
            
        measurement_noise : float
            The measurement noise covariance (R) representing sensor noise.
            Higher values mean we trust sensor readings less.
            For accelerometers, this is typically higher (0.1-1.0).
            
        dimension : int
            Number of dimensions in the accelerometer data (typically 3 for x, y, z).
        """
        self.dimension = dimension
        
        # State vector: [position, velocity]
        # For 3D accelerometer, this would be [x, y, z, vx, vy, vz]
        self.state_dim = 2 * dimension
        
        # State transition matrix (F)
        # This describes how the state evolves from t to t+1 without controls or noise
        self.F = np.eye(self.state_dim)
        for i in range(dimension):
            self.F[i, i + dimension] = 1  # Position is updated by velocity
            
        # Measurement matrix (H)
        # This describes how to map the state to a measurement
        # For accelerometer, we only measure position (acceleration), not velocity
        self.H = np.zeros((dimension, self.state_dim))
        for i in range(dimension):
            self.H[i, i] = 1
            
        # Process noise covariance (Q)
        # Represents the uncertainty in our model
        self.Q = np.eye(self.state_dim) * process_noise
        
        # Measurement noise covariance (R)
        # Represents the uncertainty in our measurements
        self.R = np.eye(dimension) * measurement_noise
        
        # Initial state
        self.x = np.zeros(self.state_dim)
        
        # Initial covariance estimate (P)
        # Represents uncertainty in our initial state
        self.P = np.eye(self.state_dim)
        
    def predict(self):
        """
        Predict the state ahead using the system model.
        
        This step projects the current state and error covariance forward
        to obtain the a priori estimates for the next time step.
        """
        # Project the state ahead
        self.x = self.F @ self.x
        
        # Project the error covariance ahead
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, measurement):
        """
        Update the state estimate with a new measurement.
        
        Parameters:
        -----------
        measurement : array_like
            New measurement vector from accelerometer (x, y, z).
            
        Returns:
        --------
        array_like
            The updated state estimate.
        """
        # Ensure measurement is the right shape
        measurement = np.array(measurement).reshape(self.dimension, 1)
        
        # Compute the Kalman gain
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update the estimate with measurement
        y = measurement - self.H @ self.x.reshape(self.state_dim, 1)
        self.x = self.x + (K @ y).flatten()
        
        # Update the error covariance
        I = np.eye(self.state_dim)
        self.P = (I - K @ self.H) @ self.P
        
        return self.x[:self.dimension]  # Return position estimate
        
    def filter_data(self, accelerometer_data):
        """
        Apply Kalman filtering to a sequence of accelerometer readings.
        
        Parameters:
        -----------
        accelerometer_data : array_like
            Array of accelerometer measurements, shape (n, 3) for 3D data.
            
        Returns:
        --------
        array_like
            Filtered accelerometer data.
        """
        filtered_data = np.zeros_like(accelerometer_data)
        
        for i, measurement in enumerate(accelerometer_data):
            self.predict()
            filtered_pos = self.update(measurement)
            filtered_data[i] = filtered_pos
            
        return filtered_data


def count_reps(filtered_data, axis=1, height=0.5, distance=20):
    """
    Count repetitions in filtered accelerometer data.
    
    Parameters:
    -----------
    filtered_data : array_like
        Filtered accelerometer data.
    axis : int
        Axis to analyze for repetitions (0=x, 1=y, 2=z).
    height : float
        Minimum height of peaks to consider as reps.
    distance : int
        Minimum number of samples between peaks.
        
    Returns:
    --------
    int
        Number of repetitions detected.
    array_like
        Indices of the repetition peaks.
    """
    # Extract data for the specific axis
    axis_data = filtered_data[:, axis]
    
    # Find peaks in the data
    peaks, _ = find_peaks(axis_data, height=height, distance=distance)
    
    return len(peaks), peaks


def calculate_rep_speed(filtered_data, peaks, sampling_rate=100):
    """
    Calculate the speed of each repetition.
    
    Parameters:
    -----------
    filtered_data : array_like
        Filtered accelerometer data.
    peaks : array_like
        Indices of repetition peaks.
    sampling_rate : float
        Number of samples per second.
        
    Returns:
    --------
    array_like
        Speed of each repetition in arbitrary units.
    """
    if len(peaks) < 2:
        return np.array([])
    
    # Calculate time between peaks (rep duration)
    rep_durations = np.diff(peaks) / sampling_rate  # in seconds
    
    # Calculate rep speed (inverse of duration)
    rep_speeds = 1 / rep_durations
    
    return rep_speeds


def detect_rest_periods(filtered_data, axis=1, threshold=0.2, min_rest_samples=50):
    """
    Detect rest periods between sets.
    
    Parameters:
    -----------
    filtered_data : array_like
        Filtered accelerometer data.
    axis : int
        Axis to analyze for rest periods (0=x, 1=y, 2=z).
    threshold : float
        Maximum acceleration magnitude to consider as rest.
    min_rest_samples : int
        Minimum number of consecutive samples below threshold to count as rest.
        
    Returns:
    --------
    list
        List of tuples (start_idx, end_idx) for each rest period.
    """
    # Calculate magnitude of acceleration for the given axis
    activity = np.abs(filtered_data[:, axis])
    
    # Find segments below threshold
    is_resting = activity < threshold
    
    rest_periods = []
    rest_start = None
    
    # Detect continuous rest periods
    for i, resting in enumerate(is_resting):
        if resting and rest_start is None:
            rest_start = i
        elif not resting and rest_start is not None:
            if i - rest_start >= min_rest_samples:
                rest_periods.append((rest_start, i))
            rest_start = None
    
    # Handle case where rest continues until the end
    if rest_start is not None and len(is_resting) - rest_start >= min_rest_samples:
        rest_periods.append((rest_start, len(is_resting)))
    
    return rest_periods


def analyze_workout(raw_data, sampling_rate=100, process_noise=0.01, measurement_noise=0.1):
    """
    Analyze workout data to extract relevant metrics.
    
    Parameters:
    -----------
    raw_data : array_like
        Raw accelerometer data, shape (n, 3).
    sampling_rate : float
        Number of samples per second.
    process_noise : float
        Process noise parameter for Kalman filter.
    measurement_noise : float
        Measurement noise parameter for Kalman filter.
        
    Returns:
    --------
    dict
        Dictionary containing workout metrics:
        - filtered_data: Filtered accelerometer data
        - rep_count: Number of repetitions detected
        - rep_peaks: Indices of repetition peaks
        - rep_speeds: Speed of each repetition
        - rest_periods: List of rest periods (start, end)
        - rest_durations: Duration of each rest period in seconds
    """
    # Initialize and apply Kalman filter
    kf = KalmanFilterAccelerometer(process_noise=process_noise, measurement_noise=measurement_noise)
    filtered_data = kf.filter_data(raw_data)
    
    # Determine the most active axis (typically y for vertical movement)
    axis_variance = np.var(filtered_data, axis=0)
    primary_axis = np.argmax(axis_variance)
    
    # Count repetitions
    rep_count, rep_peaks = count_reps(filtered_data, axis=primary_axis)
    
    # Calculate rep speed
    rep_speeds = calculate_rep_speed(filtered_data, rep_peaks, sampling_rate)
    
    # Detect rest periods
    rest_periods = detect_rest_periods(filtered_data, axis=primary_axis)
    
    # Calculate rest durations
    rest_durations = [(end - start) / sampling_rate for start, end in rest_periods]
    
    return {
        'filtered_data': filtered_data,
        'primary_axis': primary_axis,
        'rep_count': rep_count,
        'rep_peaks': rep_peaks,
        'rep_speeds': rep_speeds,
        'rest_periods': rest_periods,
        'rest_durations': rest_durations
    }


def visualize_workout_analysis(raw_data, analysis_results, sampling_rate=100):
    """
    Visualize the workout analysis results.
    
    Parameters:
    -----------
    raw_data : array_like
        Raw accelerometer data.
    analysis_results : dict
        Results from analyze_workout function.
    sampling_rate : float
        Number of samples per second.
    """
    filtered_data = analysis_results['filtered_data']
    primary_axis = analysis_results['primary_axis']
    rep_peaks = analysis_results['rep_peaks']
    rest_periods = analysis_results['rest_periods']
    
    # Create time array
    time = np.arange(len(raw_data)) / sampling_rate
    
    # Create figure
    plt.figure(figsize=(15, 10))
    
    # Plot raw and filtered data
    axis_labels = ['X', 'Y', 'Z']
    plt.subplot(3, 1, 1)
    plt.plot(time, raw_data[:, primary_axis], 'b-', alpha=0.3, label=f'Raw {axis_labels[primary_axis]}-axis')
    plt.plot(time, filtered_data[:, primary_axis], 'r-', label=f'Filtered {axis_labels[primary_axis]}-axis')
    
    # Mark repetitions
    plt.plot(time[rep_peaks], filtered_data[rep_peaks, primary_axis], 'go', label='Repetitions')
    
    # Highlight rest periods
    for start, end in rest_periods:
        plt.axvspan(time[start], time[end], color='gray', alpha=0.3)
    
    plt.title('Workout Analysis')
    plt.ylabel('Acceleration')
    plt.legend()
    
    # Plot rep speed
    if len(analysis_results['rep_speeds']) > 0:
        plt.subplot(3, 1, 2)
        rep_indices = np.arange(len(analysis_results['rep_speeds']))
        plt.bar(rep_indices, analysis_results['rep_speeds'])
        plt.title('Repetition Speed')
        plt.xlabel('Repetition Number')
        plt.ylabel('Speed (reps/sec)')
    
    # Plot rest durations
    if len(analysis_results['rest_durations']) > 0:
        plt.subplot(3, 1, 3)
        rest_indices = np.arange(len(analysis_results['rest_durations']))
        plt.bar(rest_indices, analysis_results['rest_durations'])
        plt.title('Rest Durations')
        plt.xlabel('Rest Period Number')
        plt.ylabel('Duration (seconds)')
    
    plt.tight_layout()
    plt.show()


# Example usage with synthetic data
def generate_synthetic_workout_data(num_samples=1000, num_reps=10, noise_level=0.5):
    """
    Generate synthetic workout data for testing.
    
    Parameters:
    -----------
    num_samples : int
        Number of samples to generate.
    num_reps : int
        Number of repetitions to simulate.
    noise_level : float
        Level of noise to add to the data.
        
    Returns:
    --------
    array_like
        Synthetic accelerometer data.
    """
    # Time array
    t = np.linspace(0, 10, num_samples)
    
    # Create a sine wave with varying frequency to simulate reps
    rep_signal = np.zeros_like(t)
    
    # Add repetitions with random variations
    rep_indices = np.linspace(0, len(t)-1, num_reps+2)[1:-1].astype(int)
    rep_width = len(t) // (num_reps * 3)
    
    for idx in rep_indices:
        center = idx
        # Adjust the slice to ensure it has the same size as the generated signal
        # by using min to make sure we don't exceed the array bounds
        start_idx = max(0, center - rep_width // 2)
        end_idx = min(num_samples, center + rep_width // 2)
        
        # Generate the signal for the repetition
        rep_data = np.sin(np.linspace(0, np.pi, end_idx - start_idx)) * (0.8 + 0.4 * np.random.random())
        
        # Assign the signal to the adjusted slice
        rep_signal[start_idx:end_idx] = rep_data 
    
    # Create 3D acceleration data (x, y, z)
    # Assume y is the primary axis for vertical movement
    x_accel = rep_signal * 0.3 + noise_level * np.random.randn(num_samples)
    y_accel = rep_signal + noise_level * np.random.randn(num_samples)
    z_accel = rep_signal * 0.5 + noise_level * np.random.randn(num_samples)
    
    # Combine into 3D array
    accel_data = np.column_stack((x_accel, y_accel, z_accel))
    
    return accel_data

if __name__ == "__main__":
    # Generate synthetic data
    sampling_rate = 100  # Hz
    raw_data = generate_synthetic_workout_data(num_samples=1000, num_reps=8, noise_level=0.5)
    
    # Analyze workout
    results = analyze_workout(raw_data, sampling_rate=sampling_rate,
                             process_noise=0.01, measurement_noise=0.5)
    
    # Print results
    print(f"Detected {results['rep_count']} repetitions")
    print(f"Average rep speed: {np.mean(results['rep_speeds']):.2f} reps/sec")
    print(f"Rest periods: {len(results['rest_periods'])}")
    
    # Visualize results
    visualize_workout_analysis(raw_data, results, sampling_rate)