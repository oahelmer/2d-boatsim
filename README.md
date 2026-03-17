## Equations of Motion
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