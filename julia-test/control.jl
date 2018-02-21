using StaticArrays, Rotations

function control(xᵣ, x, ωᵢ₋₁; Δt = 0.01)
 # Params
 m = 0.394
 g = 9.81
 kₚ = @SVector [1.0 1.0 2.0]
 kᵥ = @SVector [0.8 0.8 1.6]
 kᵣ = @SVector [0.1 0.1 0.1]

 # Fixed Vars
 e₃ = @SVector [0.0 0.0 1.0]'
 R = RotZYX(pose.ang...)

 # Force
 eₚ = xᵣ.pos - x.pos
 eᵥ = xᵣ.vel - x.vel
 F⃗ = kₚ*eₚ + kᵥ*eᵥ + m*g*e₃ + m*xᵣ.acc
 F = F⃗ ⋅ R[:,end]

 # Angular Rates
 b₃ = normalize(F⃗)
 ψ⃗ = [cos(x.ang[3]) sin(x.ang[3]) 0.0]'
 b₂ = normalize(b₃ × ψ⃗)
 b₁ = b₂ × b₃
 Rᵢ = [b₁ b₂ b₃]
 R̃ = Rᵢ'*R;
 eᵣ = 0.5*vee(R̃ - R̃')
 ωᵢ = vee((Rᵢ - Rᵢ₋₁)/Δt*Rᵢ')
 ω = R̃'*ωᵢ - kᵣ*eᵣ

 F, ω, Rᵢ₋₁
end

function vee(R)
 @SVector [R[3,2] R[1,3] R[2,1]]
end
