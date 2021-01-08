
using RobotOS
using PyCall
roslib = pyimport("roslib")
roslib.load_manifest("amrl_msgs")
using OneVision.ROSCar1D: ros_main

#@rosimport amrl_msgs.msg: AckermannCurvatureDriveMsg
#@rosimport nav_msgs.msg: Odometry
#rostypegen()
using BenchmarkTools: @benchmark

if ! isinteractive()
    ros_main()
else
    print("Warning - running in intreactive mode - this may result in unexpected behavior, use with caution")
    ros_main()
end


# @profview run_example(1:20 * t_end, freq; noise=0.0, plot_result=false)
# run_example(1:20 * t_end, freq; noise=0.0, plot_result=false) modules=[OneVision] maxdepth=3

# @benchmark run_example(1:20 * t_end, freq; noise=0.0, plot_result=false)
