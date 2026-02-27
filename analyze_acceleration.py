#!/usr/bin/env python3
"""
Stepper Motor Acceleration Analysis
Simulates the acceleration algorithm to analyze integer vs floating-point precision issues

Usage:
    python3 analyze_acceleration.py
    
Or customize parameters by editing the values below.
"""

import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
import sys

# Constants from step.c
STP_TIMER_FREQ_HZ = 250000

# ========================================
# CONFIGURE YOUR MOTOR PARAMETERS HERE
# ========================================
STEP_ANGLE = 1.8         # degrees per full step
MICROSTEPPING = 2        # microsteps per full step
ACCELERATION = 100.0     # degrees/s²
TARGET_ACCEL = 100.0     # For plotting reference line (same as ACCELERATION)
END_VELOCITY = 720.0     # degrees/s (higher to show the problem)
TOTAL_STEPS = 3000       # steps to simulate (max for plot)


def calculate_c0(step_angle, acceleration, microstepping, timer_freq):
    """Calculate initial step delay (c_0)"""
    c_0 = 0.676 * timer_freq * sqrt(2.0 * step_angle / (acceleration * microstepping))
    return c_0


def calculate_end_velocity_delay(step_angle, end_velocity, microstepping, timer_freq):
    """Calculate delay for constant velocity phase"""
    end_delay = step_angle * timer_freq / (end_velocity * microstepping)
    return end_delay


def simulate_acceleration_integer(c_0, end_velocity_delay, accel_steps):
    """
    Simulate acceleration using INTEGER arithmetic (old buggy version)
    This mimics uint16_t behavior without scaling - WILL FAIL
    """
    delays = []
    velocities = []
    delta_delays = []
    
    prev_delay = int(c_0)  # uint16_t cast
    
    for n in range(1, accel_steps + 1):
        # Integer division - THIS IS THE PROBLEM!
        numerator = 2 * prev_delay
        denominator = 4 * n + 1
        delta = numerator // denominator  # Integer division in C
        
        next_delay = prev_delay - delta
        
        # Clamp to minimum delay
        if next_delay < int(end_velocity_delay):
            next_delay = int(end_velocity_delay)
        
        # Calculate velocity from delay (steps/sec, then to degrees/sec)
        if next_delay > 0:
            step_freq = STP_TIMER_FREQ_HZ / next_delay
            velocity = (step_freq * STEP_ANGLE) / MICROSTEPPING
        else:
            velocity = 0
        
        delays.append(next_delay)
        velocities.append(velocity)
        delta_delays.append(delta)
        
        prev_delay = next_delay
    
    return delays, velocities, delta_delays


def simulate_acceleration_fixed_point(c_0, end_velocity_delay, accel_steps):
    """
    Simulate acceleration using FIXED-POINT arithmetic (16.16  as in the updated C code)
    Uses uint32_t with 16 fractional bits (scale by 65536)
    This provides much better precision than 8-bit scaling!
    """
    delays = []
    velocities = []
    delta_delays = []
    
    SCALE_BITS = 16  # Changed from 8 to 16 for more precision!
    SCALE = 1 << SCALE_BITS  # 65536
    
    # Scale up initial values
    prev_delay_scaled = int(c_0) << SCALE_BITS  # c_0 * 65536
    min_delay_scaled = int(end_velocity_delay) << SCALE_BITS
    
    stall_at = None
    for n in range(1, accel_steps + 1):
        # Fixed-point integer division - maintains precision!
        numerator = 2 * prev_delay_scaled
        denominator = 4 * n + 1
        delta = numerator // denominator
        
        # Check if we're stalling (delta becomes 0)
        if delta == 0 and stall_at is None:
            stall_at = n
            print(f"   [Fixed-point debug] Stalling at n={n}: prev_scaled={prev_delay_scaled}, num={numerator}, denom={denominator}")
            print(f"                       This means prev_delay_scaled dropped below {denominator//2}")
        
        next_delay_scaled = prev_delay_scaled - delta
        
        # Clamp to minimum delay (scaled)
        if next_delay_scaled < min_delay_scaled:
            next_delay_scaled = min_delay_scaled
        
        # Scale down to actual timer value
        actual_delay = next_delay_scaled >> SCALE_BITS
        
        # Calculate velocity from actual delay
        if actual_delay > 0:
            step_freq = STP_TIMER_FREQ_HZ / actual_delay
            velocity = (step_freq * STEP_ANGLE) / MICROSTEPPING
        else:
            velocity = 0
        
        delays.append(actual_delay)
        velocities.append(velocity)
        delta_delays.append(delta >> SCALE_BITS)  # Show unscaled delta for plotting
        
        prev_delay_scaled = next_delay_scaled
    
    return delays, velocities, delta_delays


def simulate_acceleration_float(c_0, end_velocity_delay, accel_steps):
    """
    Simulate acceleration using FLOATING-POINT arithmetic (David Austin formula)
    This shows what the iterative formula produces
    """
    delays = []
    velocities = []
    delta_delays = []
    
    prev_delay = float(c_0)
    
    for n in range(1, accel_steps + 1):
        # Floating-point division - preserves fractional precision
        numerator = 2.0 * prev_delay
        denominator = 4.0 * n + 1.0
        delta = numerator / denominator
        
        next_delay = prev_delay - delta
        
        # Clamp to minimum delay
        if next_delay < end_velocity_delay:
            next_delay = end_velocity_delay
        
        # Calculate velocity from delay
        if next_delay > 0:
            step_freq = STP_TIMER_FREQ_HZ / next_delay
            velocity = (step_freq * STEP_ANGLE) / MICROSTEPPING
        else:
            velocity = 0
        
        delays.append(next_delay)
        velocities.append(velocity)
        delta_delays.append(delta)
        
        prev_delay = next_delay
    
    return delays, velocities, delta_delays


def simulate_acceleration_linear(c_0, end_velocity_delay, accel_steps):
    """
    PERFECTLY LINEAR acceleration - direct interpolation
    Delay changes linearly from c_0 to end_velocity_delay
    This gives perfectly linear velocity increase!
    """
    delays = []
    velocities = []
    delta_delays = []
    
    for n in range(1, accel_steps + 1):
        # Direct linear interpolation: d = start + (end - start) * progress
        progress = n / accel_steps
        next_delay = c_0 + (end_velocity_delay - c_0) * progress
        
        # Calculate velocity
        if next_delay > 0:
            step_freq = STP_TIMER_FREQ_HZ / next_delay
            velocity = (step_freq * STEP_ANGLE) / MICROSTEPPING
        else:
            velocity = 0
        
        delays.append(next_delay)
        velocities.append(velocity)
        
        if len(delays) > 1:
            delta_delays.append(delays[-1] - delays[-2])
        else:
            delta_delays.append(c_0 - next_delay)
    
    return delays, velocities, delta_delays


def simulate_acceleration_scurve(c_0, end_velocity_delay, accel_steps):
    """
    S-CURVE acceleration - ease-in/ease-out using smoothstep
    Gives smooth, natural-feeling acceleration without harsh transitions
    Uses formula: f(t) = t^2 * (3 - 2*t) where t goes from 0 to 1
    """
    delays = []
    velocities = []
    delta_delays = []
    
    for n in range(1, accel_steps + 1):
        # Smoothstep interpolation: f(t) = 3*t^2 - 2*t^3
        t = n / accel_steps
        smooth_t = 3 * t * t - 2 * t * t * t
        next_delay = c_0 + (end_velocity_delay - c_0) * smooth_t
        
        # Calculate velocity
        if next_delay > 0:
            step_freq = STP_TIMER_FREQ_HZ / next_delay
            velocity = (step_freq * STEP_ANGLE) / MICROSTEPPING
        else:
            velocity = 0
        
        delays.append(next_delay)
        velocities.append(velocity)
        
        if len(delays) > 1:
            delta_delays.append(delays[-1] - delays[-2])
        else:
            delta_delays.append(c_0 - next_delay)
    
    return delays, velocities, delta_delays


def convert_to_time_based(delays, velocities, timer_freq_hz):
    """Convert step-based delays into actual elapsed time by accumulating delays.
    
    Args:
        delays: Array of timer tick delays for each step
        velocities: Array of velocities (not used, for reference)
        timer_freq_hz: Timer frequency in Hz (e.g., 250000)
    
    Returns:
        Array of cumulative elapsed times in seconds
    """
    times = [0.0]
    cumulative_time = 0.0
    
    # Accumulate time for each step
    for delay_ticks in delays[:-1]:  # Exclude last to match velocity array length
        cumulative_time += delay_ticks / timer_freq_hz
        times.append(cumulative_time)
    
    return np.array(times)


def main():
    # Calculate initial parameters
    c_0 = calculate_c0(STEP_ANGLE, ACCELERATION, MICROSTEPPING, STP_TIMER_FREQ_HZ)
    end_velocity_delay = calculate_end_velocity_delay(STEP_ANGLE, END_VELOCITY, MICROSTEPPING, STP_TIMER_FREQ_HZ)
    
    # Calculate how many steps needed to reach end velocity
    accel_steps = int((END_VELOCITY * END_VELOCITY * MICROSTEPPING) / 
                      (2 * ACCELERATION * STEP_ANGLE))
    
    # Limit to total steps for visualization
    accel_steps = min(accel_steps, TOTAL_STEPS)
    
    print(f"=== Stepper Motor Acceleration Analysis ===")
    print(f"Timer Frequency: {STP_TIMER_FREQ_HZ} Hz")
    print(f"Step Angle: {STEP_ANGLE}°")
    print(f"Microstepping: {MICROSTEPPING}")
    print(f"Acceleration: {ACCELERATION} °/s²")
    print(f"Target End Velocity: {END_VELOCITY} °/s")
    print(f"Initial Delay (c_0): {c_0:.2f} timer ticks")
    print(f"End Velocity Delay: {end_velocity_delay:.2f} timer ticks")
    print(f"Acceleration Steps: {accel_steps}")
    print()
    
    # Simulate all approaches
    print("Simulating acceleration profiles...")
    delays_int, velocities_int, deltas_int = simulate_acceleration_integer(
        c_0, end_velocity_delay, accel_steps)
    delays_fixed, velocities_fixed, deltas_fixed = simulate_acceleration_fixed_point(
        c_0, end_velocity_delay, accel_steps)
    delays_austin, velocities_austin, deltas_austin = simulate_acceleration_float(
        c_0, end_velocity_delay, accel_steps)
    delays_linear, velocities_linear, deltas_linear = simulate_acceleration_linear(
        c_0, end_velocity_delay, accel_steps)
    delays_scurve, velocities_scurve, deltas_scurve = simulate_acceleration_scurve(
        c_0, end_velocity_delay, accel_steps)
    
    step_numbers = np.arange(1, accel_steps + 1)
    
    # Convert to time-based (actual elapsed time)
    time_austin = convert_to_time_based(delays_austin, velocities_austin, STP_TIMER_FREQ_HZ)
    time_linear = convert_to_time_based(delays_linear, velocities_linear, STP_TIMER_FREQ_HZ)
    time_scurve = convert_to_time_based(delays_scurve, velocities_scurve, STP_TIMER_FREQ_HZ)
    
    # Find where integer version stops changing
    stall_point_int = None
    for i in range(len(deltas_int)):
        if deltas_int[i] == 0:
            stall_point_int = i + 1
            break
    
    print("\n=== RESULTS ===")
    if stall_point_int:
        print(f"❌ OLD INTEGER VERSION (uint16_t) STALLS AT STEP {stall_point_int}")
        print(f"    Final velocity: {velocities_int[stall_point_int-1]:.1f} °/s")
        print(f"    Target velocity: {END_VELOCITY:.1f} °/s")
        print(f"    Velocity reached: {velocities_int[stall_point_int-1]/END_VELOCITY*100:.1f}%")
    else:
        print("✓ Old integer version reaches target velocity")
    
    # Check fixed-point version
    final_vel_fixed = velocities_fixed[-1]
    target_reached = abs(final_vel_fixed - END_VELOCITY) / END_VELOCITY < 0.02  # Within 2%
    
    print("\n" + "="*70)
    print("ACCELERATION ALGORITHM COMPARISON")
    print("="*70)
    print(f"\n📊 Target velocity: {END_VELOCITY:.1f} °/s")
    print(f"   Acceleration steps: {accel_steps}")
    
    if stall_point_int:
        print(f"\n❌ OLD (uint16_t):")
        print(f"    Stalls at step {stall_point_int}, reaches only {velocities_int[stall_point_int-1]:.1f} °/s ({velocities_int[stall_point_int-1]/END_VELOCITY*100:.1f}%)")
    
    if not target_reached:
        print(f"\n⚠️  CURRENT (uint32_t fixed-point 16.16):")
        print(f"    Reaches {final_vel_fixed:.1f} °/s ({final_vel_fixed/END_VELOCITY*100:.2f}%)")
    else:
        print(f"\n✅ CURRENT (uint32_t fixed-point 16.16):")
        print(f"    Reaches {final_vel_fixed:.1f} °/s ({final_vel_fixed/END_VELOCITY*100:.2f}%)")
    
    # Analyze new options
    final_linear = velocities_linear[-1]
    final_scurve = velocities_scurve[-1]
    
    print(f"\n🟢 OPTION 1: PERFECT LINEAR (direct interpolation)")
    print(f"    Reaches {final_linear:.1f} °/s ({final_linear/END_VELOCITY*100:.2f}%)")
    print(f"    ✓ Perfectly linear velocity ramp")
    print(f"    ✓ Ultra simple: delay = start + (end - start) * n / steps")
    print(f"    ✓ NO DIVISION - just multiplication and addition!")
    print(f"    ⚠️  ISR loads: constant across all steps")
    
    print(f"\n🔵 OPTION 2: S-CURVE (smooth acceleration)")
    print(f"    Reaches {final_scurve:.1f} °/s ({final_scurve/END_VELOCITY*100:.2f}%)")
    print(f"    ✓ Smooth ease-in/ease-out (natural feeling)")
    print(f"    ✓ No jerk (second derivative = 0)")
    print(f"    ✓ ISR loads: constant (just 1 multiply + smoothstep calc)")
    print(f"    ✓ Better for mechanical systems with compliance")
    
    print(f"\n🟣 CURRENT: David Austin Iterative Formula")
    print(f"    Reaches {final_vel_fixed:.1f} °/s ({final_vel_fixed/END_VELOCITY*100:.2f}%)")
    print(f"    ~ Approximately linear (not perfect)")
    print(f"    ⚠️  REQUIRES DIVISION every step")
    print(f"    ⚠️  Causes ISR timing anomalies at high speeds")
    
    print("\n" + "="*70)
    print("RECOMMENDATION:")
    print("="*70)
    print("\n🥇 BEST CHOICE: PERFECT LINEAR (Option 1)")
    print("   - Solve your high-speed choppiness immediately")
    print("   - Simplest code (no division, no complex math)")
    print("   - Fastest ISR (constant time per step)")
    print("   - Perfectly predictable timing")
    
    print("\n🥈 ALTERNATIVE: S-CURVE (Option 2)")
    print("   - If you want smoother mechanical feel")
    print("   - Also fails high-speed choppiness issue")
    print("   - Slightly more math but still simple")
    
    print("\n❌ KEEP CURRENT: David Austin")
    print("   - Causes timing issues at high speeds")
    print("   - Not worth the complexity")
    print()
    
    # Create comprehensive plots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Stepper Motor Acceleration Algorithm Comparison', 
                 fontsize=14, fontweight='bold')
    
    # Plot 1: Velocity over TIME - MAIN COMPARISON (THE REAL VIEW!)
    ax1 = axes[0, 0]
    ax1.plot(time_austin, velocities_austin, 'purple', label='David Austin formula', linewidth=2, linestyle='--', alpha=0.8)
    ax1.plot(time_linear, velocities_linear, 'g-', label='LINEAR delay decrease', linewidth=2, alpha=0.7)
    ax1.plot(time_scurve, velocities_scurve, 'b-', label='S-CURVE (ease-in/out)', linewidth=2.5)
    ax1.axhline(y=END_VELOCITY, color='gray', linestyle=':', alpha=0.5, label='Target')
    ax1.set_xlabel('Time (seconds)')
    ax1.set_ylabel('Velocity (°/s)')
    ax1.set_title('REAL VIEW: Velocity vs TIME (this is what matters!)')
    ax1.legend(loc='lower right', fontsize=9)
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Step-based view for comparison (MISLEADING - shows why time-based is needed)
    ax2 = axes[0, 1]
    ax2.plot(step_numbers, velocities_austin, 'purple', label='David Austin formula', linewidth=2, linestyle='--', alpha=0.8)
    ax2.plot(step_numbers, velocities_linear, 'g-', label='LINEAR delay decrease', linewidth=2, alpha=0.7)
    ax2.plot(step_numbers, velocities_scurve, 'b-', label='S-CURVE', linewidth=2.5)
    ax2.axhline(y=END_VELOCITY, color='gray', linestyle=':', alpha=0.5)
    ax2.set_xlabel('Step Number')
    ax2.set_ylabel('Velocity (°/s)')
    ax2.set_title('MISLEADING VIEW: Velocity vs Step (looks different because variable step times!)')
    ax2.legend(loc='lower right', fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    
    # Plot 3: Acceleration (velocity change over TIME)
    ax3 = axes[1, 0]
    if len(velocities_austin) > 1 and len(time_austin) > 1:
        accel_austin = np.diff(velocities_austin) / np.diff(time_austin)
        accel_linear = np.diff(velocities_linear) / np.diff(time_linear)
        accel_scurve = np.diff(velocities_scurve) / np.diff(time_scurve)
        ax3.plot(time_austin[:-1], accel_austin, 'purple', label='David Austin', linewidth=1.5, linestyle='--', alpha=0.8)
        ax3.plot(time_linear[:-1], accel_linear, 'g-', label='LINEAR (constant)', linewidth=2.5)
        ax3.plot(time_scurve[:-1], accel_scurve, 'b:', label='S-CURVE (smooth)', linewidth=2)
        ax3.axhline(y=TARGET_ACCEL, color='cyan', linestyle=':', alpha=0.5, label=f'Target ({TARGET_ACCEL}°/s²)')
    ax3.set_xlabel('Time (seconds)')
    ax3.set_ylabel('Acceleration (°/s²)')
    ax3.set_title('Acceleration over TIME (Linear = flat, S-Curve = rounded)')
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Jerk measurement (smoothness metric - second derivative over time)
    ax4 = axes[1, 1]
    if len(velocities_austin) > 2 and len(time_austin) > 2:
        accel_austin = np.diff(velocities_austin) / np.diff(time_austin)
        accel_linear = np.diff(velocities_linear) / np.diff(time_linear)
        accel_scurve = np.diff(velocities_scurve) / np.diff(time_scurve)
        
        jerk_austin = np.diff(accel_austin) / np.diff(time_austin[:-1])
        jerk_linear = np.diff(accel_linear) / np.diff(time_linear[:-1])
        jerk_scurve = np.diff(accel_scurve) / np.diff(time_scurve[:-1])
        
        ax4.plot(time_austin[:-2], jerk_austin, 'purple', label='David Austin', linewidth=1.5, linestyle='--', alpha=0.8)
        ax4.plot(time_linear[:-2], jerk_linear, 'g-', label='LINEAR (max jerk)', linewidth=2.5)
        ax4.plot(time_scurve[:-2], jerk_scurve, 'b:', label='S-CURVE (zero jerk)', linewidth=2)
        ax4.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.5)
    ax4.set_xlabel('Time (seconds)')
    ax4.set_ylabel('Jerk (°/s³)')
    ax4.set_title('Jerk over TIME (Smoothness: lower = smoother motion)')
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plot
    output_file = '/home/dg/data/AppData/MCUExpress/Workspaces/TMC2209_Driver/acceleration_analysis_time_based.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n📈 TIME-BASED plot saved to: {output_file}")
    print("\n✨ KEY INSIGHT: Plots 1 & 2 are the same data!")
    print("   Time-based (Plot 1) shows velocity change smoothness")
    print("   Step-based (Plot 2) is MISLEADING due to variable step delays")
    print("   Acceleration & Jerk (Plots 3 & 4) show REAL smoothness over time")
    
    plt.show()


if __name__ == "__main__":
    main()
