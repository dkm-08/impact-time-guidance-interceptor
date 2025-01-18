# impact-time-guidance-interceptor

A nonlinear guidance law for precise impact time control of interceptors, featuring closed-form solutions and cooperative guidance for salvo attacks. Built in MATLAB/Simulink, enabling multiple interceptors to achieve simultaneous impact timing.

## 🎯 Features

- Nonlinear guidance law with exponential heading angle variation
- Closed-form expressions for design parameters and impact time
- Cooperative guidance scheme for salvo attacks
- High precision in achieving desired impact times
- Robust performance in multiple interceptor scenarios

## 🔧 Prerequisites

- MATLAB R2022a
- Simulink
- Parallel Computing Toolbox (MATLAB)

## 💻 Implementation

The guidance law is implemented in two scenarios:

### Ideal Scenario
```matlab
% Open-loop control strategy
1. Compute tf_min and tf_max
2. Select tf from (tf_min, tf_max]
3. Find f and b parameters
4. Generate lateral acceleration command
```

### Salvo Attack
```matlab
% Multiple interceptor coordination
1. Compute individual tf_min and tf_max for each interceptor
2. Determine permissible tf set
3. Select common tf from intersection set
4. Calculate control parameters for each interceptor
5. Generate and execute acceleration commands
```

## 📊 Results

- Accurate achievement of desired impact times
- Successful coordination in salvo attacks
- Efficient trajectory generation
- Minimal control effort requirements

## 🚀 Future Work

- [ ] Implementation of autopilot lag compensation
- [ ] Extension to maneuvering targets
- [ ] Integration of additional guidance objectives
- [ ] Hardware-in-the-loop simulations
- [ ] Performance optimization studies


## 👤 Author

**Dhanush K Mukundan**  
