## overview

This is a simiulator game where you can race two boats.
You drive your boat by tilting the motor with the left and right arrow keys, and accelerate using forward and backward keys.

Your competitors have two different control systems. 
1. the slower one is a LOS guidance system using a PD regulator for path following

2. the faster one is a Reinforcement learning AI that is trained to drive the boat to follow a given path, so this is the nonlinear alternative to the other boat. 


## to run the code with the ai I trained:

Go to the root directiory of the repository and create your own build folder by running:

```
mkdir build
```

```
cd build
```

build the project with cmake and make using:

```
cmake ..
```

```
make
```

run the simulation by running:

```
./BoatSim
```

This should run the simulator using the trained boat_policy.onnx and boat_policy.zip


## Training the ai youself

Train the AI by running the RLtaining.py file in src, this takes a while. For this, the onnxruntime package is used from src. If this dont work you can install onnxruntime yourself from https://github.com/microsoft/onnxruntime/releases install the version from your system and extract the folder into the src of this project for easy use. The training will be finished when 
|    total_timesteps      | 503808      |
is showing.

## Equations of Motion of the simulator

The simulator simulates boats in an x y plane with a theta angle of the boat and a phi motor angle relative to the boat angle. and the motor is places some distance L away from the CM.

The generalized coordinates are

$$
q = [x , v_x , y , v_y , \theta , \omega , \phi , \dot{\phi}]^T
$$

So our system

$$
\dot{q} = f(q)
$$

becomes

$$
\frac{d}{dt} \begin{bmatrix} x \\ v_x \\ y \\ v_y \\ \theta \\ \omega \\ \phi \\ \dot{\phi} \end{bmatrix} = 
\begin{bmatrix} 
v_x \\ 
\frac{1}{M} (F \sin(\theta + \phi) - D_v v_x) \\ 
v_y \\ 
\frac{1}{M} (F \cos(\theta + \phi) - D_v v_y) \\ 
\omega \\ 
\frac{1}{I} (L \cdot F \sin(\phi) - D_\omega \omega) \\ 
\dot{\phi} \\ 
\frac{1}{I_m} (\tau - D_\phi \dot{\phi}) 
\end{bmatrix}
$$

This is simulated using RK4 in boat.cpp

