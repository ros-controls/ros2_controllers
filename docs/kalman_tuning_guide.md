# Kalman Filter Tuning Guide for Hydraulic Systems

## Quick Reference

### R Matrix (Measurement Noise)
- **Too small R**: Filter tracks measurements exactly, noisy output
- **Too large R**: Filter ignores measurements, sluggish response

### Q Matrix (Process Noise)
- **Too small Q**: Filter trusts model too much, slow adaptation
- **Too large Q**: Filter doesn't trust model, follows measurements blindly

## Tuning Flowchart

```
Start: Q = 1e-9·I, R = measured_variance

         ┌─────────────────┐
         │ Run filter      │
         └────────┬────────┘
                  │
         ┌────────▼────────┐
         │ Check estimate  │
         └────────┬────────┘
                  │
         ┌────────▼────────────────────────┐
         │ Is estimate too noisy/jittery?  │
         └────────┬────────────────────────┘
                  │
         Yes ◄────┴────► No
          │              │
   ┌──────▼───────┐     │
   │ Decrease Q   │     │
   │ or           │     │
   │ Increase R   │     │
   └──────┬───────┘     │
          │             │
          └─────────────┤
                        │
         ┌──────────────▼─────────────────┐
         │ Is estimate too slow/lagging?  │
         └──────────────┬─────────────────┘
                        │
                Yes ◄───┴───► No
                 │            │
          ┌──────▼───────┐   │
          │ Increase Q   │   │
          │ or           │   │
          │ Decrease R   │   │
          └──────┬───────┘   │
                 │           │
                 └───────────┤
                             │
                    ┌────────▼────────┐
                    │ Filter is tuned!│
                    └─────────────────┘
```

## Metrics to Monitor

### 1. Innovation Sequence
```
innovation = measurement - predicted_measurement

Should be:
- Zero-mean (no bias)
- Gaussian distributed
- Variance ≈ R
```

If innovation variance >> R → Q is too small
If innovation variance << R → Q is too large

### 2. Estimation Error Covariance (P matrix)

Plot diagonal elements of P over time:
- Should converge to steady-state
- If growing → filter diverging (Q too large)
- If shrinking to zero → over-confident (R too large)

### 3. Normalized Innovation Squared (NIS)

```
NIS = innovationᵀ·S⁻¹·innovation

where S = CPCᵀ + R

Should follow χ² distribution with n degrees of freedom
(n = number of measurements)
```

## Example Values for Hydraulic Mobile Base

```yaml
# Encoder specs: 2048 CPR, 0.15m radius
# Quantization: ±0.0005 m
measurement_noise_std: 0.0005  # m
R_matrix: [2.5e-7, 2.5e-7]     # variance = std²

# Hydraulic system: τ = 0.6s, ±5% pressure variation
process_noise_std: 0.025        # m/s
Q_matrix: [6.25e-4, 6.25e-4, 1e-4, 1e-4]
#          └─ velocity states  └─ pressure states
```

## Automatic Tuning Methods

### Adaptive Kalman Filter
Update Q and R online based on innovation statistics:

```cpp
// Compute innovation covariance from recent history
Eigen::MatrixXd S_empirical = compute_innovation_covariance(innovation_history);

// Update R to match empirical innovation covariance
R = S_empirical - C * P * C.transpose();
```

### Maximum Likelihood Estimation
Use recorded data to find Q and R that maximize likelihood:

```python
from scipy.optimize import minimize

def negative_log_likelihood(params, data):
    Q, R = unpack_params(params)
    kf = KalmanFilter(Q=Q, R=R)
    return -kf.loglikelihood(data)

result = minimize(negative_log_likelihood, initial_guess, args=(recorded_data,))
Q_opt, R_opt = unpack_params(result.x)
```

## Practical Tips

1. **Start conservative**: Small Q, measured R
2. **Tune Q first**: R is usually well-defined by sensors
3. **Log innovation**: Save innovation sequence for offline analysis
4. **Plot covariance**: Monitor P matrix convergence
5. **Test extremes**: Try very noisy environments and sudden changes
6. **Document**: Record final Q/R values and test conditions
