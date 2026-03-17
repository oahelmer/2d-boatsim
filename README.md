# Equations of motion

Let s = (x, vx, y, vy, θ, ω, φ, φ̇)^T. Inputs: F, τ. Constants: M, I, I_MOTOR, L, DAMPING_V, DAMPING_W, DAMPING_PHI.

ẋ = vx
v̇x = (F * sin(θ + φ) - DAMPING_V * vx) / M
ẏ = vy
v̇y = (F * cos(θ + φ) - DAMPING_V * vy) / M
θ̇ = ω
ω̇ = (L * F * sin(φ) - DAMPING_W * ω) / I
φ̇ = φ̇
φ̈ = (τ - DAMPING_PHI * φ̇) / I_MOTOR

Compact vector form:
ṡ = [
  vx,
  (F*sin(θ+φ) - DAMPING_V*vx)/M,
  vy,
  (F*cos(θ+φ) - DAMPING_V*vy)/M,
  ω,
  (L*F*sin(φ) - DAMPING_W*ω)/I,
  φ̇,
  (τ - DAMPING_PHI*φ̇)/I_MOTOR
]^T
