package modules.stateblocks.ins

import modules.stateblocks.ins.MotionModelAuxData
import scorpion.filters.sensor.containers.StateBlock
import golem.*
import golem.containers.*
import golem.matrix.*
import scorpion.filters.sensor.containers.Dynamics


/**
 * A 9-state representation of an aircraft motion model using coordinated turns.
 * States: Pn       North(NED)
 *         Pe       East (NED)
 *         Vg       Ground Speed
 *         Chi      Course Angle
 *         Wn       Wind North
 *         We       Wind East
 *         Psi      Yaw
 *         Alt      Altitude (AGL)
 *         Alt_vv   Altitude Vertical Velocity
 *
 * @param label The label uniquely identifying this particular set of states.
 * @param imuModel An instance of ImuModel specifying the grade of the INS being used
 * @param curTime The time to initialize this block to.
 */
class MotionModelBlock(override var label: String) : StateBlock {
    override var numStates: Int = 9 //Is the motion model states or aux data states???????????????????????????

    private var auxData: MotionModelAuxData? = null

    /**
     * This block receives Motion Model inputs data via calls to this function. The parameter must be of type
     * [MotionModelAuxData], which is a container passing in motion model inputs
     */
    override fun receiveAuxData(auxData: Any) {
        if (auxData is MotionModelAuxData) {
            this.auxData = auxData
        } else {
            throw UnsupportedOperationException("This class only accepts MotionModelAux instances")
        }
    }

    override fun generateDynamics(xhat: Matrix<Double>, timeFrom: Time, timeTo: Time): Dynamics {


        var aux = this.auxData ?: throw UnsupportedOperationException("Error in MotionModel Aux Data")
        var airspeed = aux.airspeed
        var pitchrate = aux.pitchrate
        var yawrate = aux.yawrate
        var roll = aux.roll
        var pitch = aux.pitch
        var tau_vv:Double = 2.0//time constant on alt_vv
        var sigma_vv = 5.0 //sigma on alt_vv

        val dt = timeTo.time - timeFrom.time

        val F = generateFMotionModel(airspeed = airspeed,
                pitchrate = pitchrate,
                yawrate = yawrate,
                roll = roll,
                pitch = pitch,
                tau_vv = tau_vv,
                xhat = xhat)
        val Q = generateQMotionModel(tau_vv = tau_vv, sigma_vv = sigma_vv)
        var Phi = expm(F * dt)
        var f = { x: Matrix<Double> -> Phi * x }
        // Second order approximation
        var Qd = (Phi * Q * Phi.T + Q) * dt / 2

        return Dynamics(f, Phi, Qd)
    }

    /**
     * Takes an instance of the [IMUModel] class and generates the continuous time covariance matrix Q(t) = E[w(t)w(t)^T]
     * describing the states.
     */
    fun generateQMotionModel(tau_vv: Double, sigma_vv: Double): Matrix<Double> {

        var q = zeros(1, 9)
        q[0..7,0..7]=eye(8)*.01    //add noise to states for now need to figureout how to calculate input noise
        q[0, 0..8] = zeros(1, 9)
        q[8, 8] = 2*pow(sigma_vv,2)/tau_vv

        //add q for all states
        return q
        //return fill(rows = 15, cols = 15) { row, col -> if (row == col) pow(q[row], 2) else 0.0 }
    }

    //@formatter:off

    /**
     * Takes in the vehicles current position, velocity, and specific force and calculates the continuous time dynamics matrix F
     * such that x_dot = Fx + w
     *
     * @param airspeed in m/s
     * @param pitchrate in rad/s
     * @param yawrate in rad/s
     * @param roll in rads
     * @param pitch in rads
     */
    fun generateFMotionModel(airspeed: Double,
                             pitchrate: Double,
                             yawrate: Double,
                             roll: Double,
                             pitch: Double,
                             tau_vv: Double,
                             xhat: Matrix<Double>): Matrix<Double> {

        //Current states
        //var Pn  = xhat[0]
        //var Pe  = xhat[1]
        var Vg  = xhat[2]
        var chi = xhat[3]
        var Wn  = xhat[4]
        var We  = xhat[5]
        var psi = xhat[6]

        val g       = 9.81 //Gravity m/s
       //Inputs
        var Va = airspeed
        var q = pitchrate
        var r = yawrate
        var phi = roll
        var theta = pitch



        // Fain Motion model Below
        // Beard & McClain Motion Model
        val F = zeros(8, 8)
        var psi_dot = q*sin(phi)/cos(theta)+r*cos(phi)/cos(theta)
        var Vg_dot = (Va*cos(phi)+Wn)*(-Va*psi_dot*sin(psi)+Va*sin(phi)+We)*(Va*psi_dot*cos(psi))/Vg



        F[0..1,2..3] = mat[cos(chi), -Vg*sin(chi) end
                            sin(chi), Vg*cos(chi)]
        F[2,2]       = -Vg_dot/Vg
        F[2,4]       = -psi_dot*Va*sin(psi) / Vg
        F[2,5]       =  psi_dot*Va*cos(psi) / Vg_dot
        F[2,6]       = -psi_dot*Va*(Wn*cos(psi)+We*sin(psi)) / Vg
        F[3,2]       = g*tan(phi)*cos(chi-psi) / pow(Vg,2)
        F[3,3]       = -g*tan(phi)*sin(chi-psi) / Vg
        F[3,6]       = g*tan(phi)*sin(chi-psi) / Vg
        F[7,8]       = 1
        F[8,8]      = -1/tau_vv //time constant for vertical velocity

        return F
    }
    //@formatter:on

}