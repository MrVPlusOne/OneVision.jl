using Makie
using AbstractPlotting
AbstractPlotting.inline!(false)

scene = Scene()

points = [Point2f0(cos(t), sin(t)) for t in LinRange(0, 2pi, 20)]
colors = 1:20
scatter!(scene, points, color = colors, markersize = 15)

scene[end].markersize = 30

xs = -pi:0.01:pi
frequency = Node(3.0) # Node === Observable
phase = Node(0.0)

ys = lift(frequency, phase) do fr, ph
    @. 0.3 * sin(fr * xs - ph)
end

ys = @lift @. 0.3 * sin($frequency * xs - $phase)

lines!(scene, xs, ys, color = :blue, linewidth = 3)

frequency[] = 5.0

framerate = 30
timestamps = 0:1/framerate:3

delete!(scene, scene[end])

scene.camera

mp_x = @lift $(scene.events.mouseposition)[1]
mp_y = @lift $(scene.events.mouseposition)[2]

xlims!(scene, mp_x)
plot!(scene; )

record(scene, "phase_animation.mp4", timestamps; framerate = framerate, sleep=false) do t
    phase[] = 2 * t * 2pi
end


scene