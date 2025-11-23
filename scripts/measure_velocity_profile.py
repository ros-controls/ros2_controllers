#!/usr/bin/env python3
"""
Script to measure velocity profile characteristics of a mobile base.
Determines max velocity, acceleration, and jerk from step response.

Usage:
  ros2 run diff_drive_controller measure_velocity_profile.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
import time


class VelocityProfileMeasurement(Node):
    def __init__(self):
        super().__init__('velocity_profile_measurement')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Data storage
        self.timestamps = []
        self.velocities_linear = []
        self.velocities_angular = []

        self.start_time = None
        self.test_running = False

    def odom_callback(self, msg):
        """Record velocity data"""
        if not self.test_running:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.start_time is None:
            self.start_time = current_time

        self.timestamps.append(current_time - self.start_time)
        self.velocities_linear.append(msg.twist.twist.linear.x)
        self.velocities_angular.append(msg.twist.twist.angular.z)

    def run_step_test(self, target_velocity=0.5, duration=10.0):
        """
        Send step command and record response

        Args:
            target_velocity: Target velocity (m/s)
            duration: Test duration (seconds)
        """
        self.get_logger().info(f'Starting step test to {target_velocity} m/s...')

        # Reset data
        self.timestamps = []
        self.velocities_linear = []
        self.velocities_angular = []
        self.start_time = None
        self.test_running = True

        # Wait for initial steady state
        time.sleep(2.0)

        # Send step command
        cmd = Twist()
        cmd.linear.x = target_velocity

        start = time.time()
        while (time.time() - start) < duration:
            self.cmd_pub.publish(cmd)
            time.sleep(0.02)  # 50 Hz

        # Stop
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)

        self.test_running = False
        self.get_logger().info('Test complete. Analyzing data...')

        return self.analyze_data()

    def analyze_data(self):
        """
        Compute velocity profile characteristics from recorded data

        Returns:
            dict with keys: v_max, a_max, j_max, tau (time constant)
        """
        if len(self.timestamps) < 10:
            self.get_logger().error('Insufficient data collected!')
            return None

        t = np.array(self.timestamps)
        v = np.array(self.velocities_linear)

        # Smooth velocity data with Savitzky-Golay filter
        if len(v) > 51:
            v_smooth = savgol_filter(v, window_length=51, polyorder=3)
        else:
            v_smooth = v

        # Compute derivatives
        dt = np.diff(t)
        dv = np.diff(v_smooth)
        acceleration = dv / dt

        # Smooth acceleration
        if len(acceleration) > 51:
            a_smooth = savgol_filter(acceleration, window_length=51, polyorder=3)
        else:
            a_smooth = acceleration

        # Compute jerk (derivative of acceleration)
        da = np.diff(a_smooth)
        dt_a = dt[:-1]  # One less element due to second diff
        jerk = da / dt_a

        # Measurements
        v_max = np.max(np.abs(v_smooth))
        a_max = np.max(np.abs(a_smooth))
        j_max = np.max(np.abs(jerk))

        # Estimate time constant (time to reach 63.2% of steady state)
        v_steady = v_smooth[-100:].mean()  # Average of last 100 samples
        target_v = 0.632 * v_steady

        try:
            idx_63 = np.where(v_smooth >= target_v)[0][0]
            tau = t[idx_63] - t[0]
        except IndexError:
            tau = None
            self.get_logger().warn('Could not estimate time constant')

        # Rise time (10% to 90%)
        v_10 = 0.1 * v_steady
        v_90 = 0.9 * v_steady

        try:
            idx_10 = np.where(v_smooth >= v_10)[0][0]
            idx_90 = np.where(v_smooth >= v_90)[0][0]
            rise_time = t[idx_90] - t[idx_10]
        except IndexError:
            rise_time = None

        results = {
            'v_max': v_max,
            'a_max': a_max,
            'j_max': j_max,
            'tau': tau,
            'rise_time': rise_time,
            'steady_state_velocity': v_steady
        }

        # Plot results
        self.plot_results(t, v_smooth, acceleration, jerk, results)

        return results

    def plot_results(self, t, v, a, j, results):
        """Generate plots of velocity, acceleration, and jerk"""
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))

        # Velocity plot
        axes[0].plot(t, v, 'b-', linewidth=2, label='Measured')
        axes[0].axhline(y=results['steady_state_velocity'],
                       color='r', linestyle='--', label='Steady State')
        if results['tau'] is not None:
            axes[0].axvline(x=results['tau'],
                           color='g', linestyle='--',
                           label=f'τ = {results["tau"]:.3f}s')
        axes[0].set_ylabel('Velocity (m/s)', fontsize=12)
        axes[0].set_title('Velocity Profile Measurement', fontsize=14, fontweight='bold')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # Acceleration plot
        t_accel = t[:-1]  # One less point due to diff
        axes[1].plot(t_accel, a, 'r-', linewidth=2)
        axes[1].axhline(y=results['a_max'],
                       color='k', linestyle='--',
                       label=f'Max = {results["a_max"]:.3f} m/s²')
        axes[1].axhline(y=-results['a_max'], color='k', linestyle='--')
        axes[1].set_ylabel('Acceleration (m/s²)', fontsize=12)
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        # Jerk plot
        t_jerk = t_accel[:-1]  # One less point due to second diff
        axes[2].plot(t_jerk, j, 'g-', linewidth=2)
        axes[2].axhline(y=results['j_max'],
                       color='k', linestyle='--',
                       label=f'Max = {results["j_max"]:.3f} m/s³')
        axes[2].axhline(y=-results['j_max'], color='k', linestyle='--')
        axes[2].set_ylabel('Jerk (m/s³)', fontsize=12)
        axes[2].set_xlabel('Time (s)', fontsize=12)
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('velocity_profile_measurement.png', dpi=300)
        self.get_logger().info('Plot saved to velocity_profile_measurement.png')
        plt.show()

    def print_results(self, results):
        """Print measurement results"""
        print('\n' + '='*60)
        print('VELOCITY PROFILE MEASUREMENT RESULTS')
        print('='*60)
        print(f'Maximum Velocity:        {results["v_max"]:.4f} m/s')
        print(f'Maximum Acceleration:    {results["a_max"]:.4f} m/s²')
        print(f'Maximum Jerk:            {results["j_max"]:.4f} m/s³')
        if results['tau'] is not None:
            print(f'Time Constant (τ):       {results["tau"]:.4f} s')
        if results['rise_time'] is not None:
            print(f'Rise Time (10-90%):      {results["rise_time"]:.4f} s')
        print(f'Steady State Velocity:   {results["steady_state_velocity"]:.4f} m/s')
        print('='*60)

        print('\nRECOMMENDED CONTROLLER PARAMETERS:')
        print('='*60)
        # Conservative values (60% of measured)
        print(f'max_velocity:       {0.6 * results["v_max"]:.3f}')
        print(f'max_acceleration:   {0.6 * results["a_max"]:.3f}')
        print(f'max_jerk:           {0.6 * results["j_max"]:.3f}')
        print('='*60 + '\n')


def main(args=None):
    rclpy.init(args=args)

    node = VelocityProfileMeasurement()

    print('\n' + '='*60)
    print('VELOCITY PROFILE MEASUREMENT TOOL')
    print('='*60)
    print('This tool will:')
    print('1. Send step velocity command to your robot')
    print('2. Record actual velocity response')
    print('3. Compute max velocity, acceleration, and jerk')
    print('4. Recommend controller parameters')
    print('\nMAKE SURE:')
    print('- Robot has clear space (at least 5 meters)')
    print('- Emergency stop is accessible')
    print('- /cmd_vel and /odom topics are active')
    print('='*60 + '\n')

    input('Press ENTER to start test...')

    # Run test with different target velocities
    test_velocities = [0.3, 0.5, 0.7]  # m/s
    all_results = []

    for target_v in test_velocities:
        results = node.run_step_test(target_velocity=target_v, duration=8.0)
        if results:
            all_results.append(results)
            node.print_results(results)

        # Wait between tests
        if target_v != test_velocities[-1]:
            print('\nWaiting 5 seconds before next test...')
            time.sleep(5.0)

    # Compute conservative values across all tests
    if all_results:
        print('\n' + '='*60)
        print('FINAL RECOMMENDED PARAMETERS (across all tests):')
        print('='*60)
        v_max_overall = max([r['v_max'] for r in all_results])
        a_max_overall = max([r['a_max'] for r in all_results])
        j_max_overall = max([r['j_max'] for r in all_results])

        print(f'max_velocity:       {0.6 * v_max_overall:.3f}  # 60% of measured')
        print(f'max_acceleration:   {0.6 * a_max_overall:.3f}  # 60% of measured')
        print(f'max_jerk:           {0.6 * j_max_overall:.3f}  # 60% of measured')
        print('='*60 + '\n')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
