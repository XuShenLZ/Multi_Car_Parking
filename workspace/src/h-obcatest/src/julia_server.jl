#!/usr/bin/env julia

using RobotOS
@rosimport parking.srv: maneuver
@rosimport parking.msg: car_state, car_input
rostypegen()
using parking.srv
using parking.msg

D = Dict(["U", "L", "F"] => 1)

A = [1 2;1.5 2.5]

function handle_request(req)
    println(req)
    request = [req.Start_lane, req.End_spot, req.End_pose]
    println(haskey(D, request))
    println(D[request])

    println("returning value")
    val0 = true
    val1 = car_state()
	val1.x = A[1, :]
	val1.y = [1.1, 2.1]
	val1.psi = [1.2, 2.2]
	val1.v = [1.3, 2.3]
	val2 = car_input()
	val2.delta = [3.0, 4.0]
	val2.acc = [3.1, 4.1]
	val3 = [5.0, 6.0]
	respond = [val0, val1, val2, val3]
	return respond
end

function ros_server()
    init_node("julia_server")
    s = Service{maneuver}("park_maneuver", handle_request)
    println("ready to send maneuver")
    spin()

end

ros_server()