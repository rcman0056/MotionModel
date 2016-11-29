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
        val Qd_mm = generateQd_mmMotionModel(dt = dt,
                airspeed = airspeed,
                pitchrate = pitchrate,
                yawrate = yawrate,
                roll = roll,
                pitch = pitch,
                xhat=xhat,
                tau_vv = tau_vv,
                sigma_vv = sigma_vv)
        var Phi = expm(F * dt)





        //X_new=X_dot *dt +x_old
        //Right now f_mm returns X_dot*dt
        var f_mm = generatef_mmMotionModel(dt =dt,
                airspeed = airspeed,
                pitchrate = pitchrate,
                yawrate = yawrate,
                roll = roll,
                pitch = pitch,
                xhat=xhat)
        // Second order approximation of Vertical Velocity Noise
        var Q_vv = mat[0 , 0 end
                       0, 2*sigma_vv*sigma_vv/tau_vv]
        var Qd_vv = (Phi[7..8,7..8] * Q_vv * Phi[7..8,7..8].T + Q_vv) * dt / 2

        //Calculate f function
        var f = { x: Matrix<Double> ->

               x[7..8,0]= Phi[7..8,7..8] * x[7..8,0]
               x[0..6,0]= f_mm + x[0..6,0]
               x
        }


        //Combine Qd
        var Qd = zeros(9,9)
        Qd[0..6,0..6] = Qd_mm
        Qd[7..8,7..8] = Qd_vv
        return Dynamics(f, Phi, Qd)
    }

    /**
     * Takes an instance of the [IMUModel] class and generates the continuous time covariance matrix Q(t) = E[w(t)w(t)^T]
     * describing the states.
     */

    fun generatef_mmMotionModel(dt: Double,
                              airspeed: Double,
                              pitchrate: Double,
                              yawrate: Double,
                              roll: Double,
                              pitch: Double,
                              xhat: Matrix<Double>): Matrix<Double> {

        //current states
        var Vg   = xhat[2]
        var chi  = xhat[3]
        var Wn   = xhat[4]
        var We   = xhat[5]
        var psi  = xhat[6]
        val g    = 9.81 //Gravity m/s

        //Inputs
        var Va = airspeed
        var q = pitchrate
        var r = yawrate
        var phi = roll
        var theta = pitch

        var f_mm = zeros(7,1)
        var psi_dot = q*sin(phi)/cos(theta)+r*cos(phi)/cos(theta)
        var Vg_dot = (Va*cos(phi)+Wn)*(-Va*psi_dot*sin(psi)+Va*sin(phi)+We)*(Va*psi_dot*cos(psi))/Vg

        f_mm[0] = Vg*cos(chi)*dt
        f_mm[1] = Vg*sin(chi)*dt
        f_mm[2] = Vg_dot*dt
        f_mm[3] = g*tan(phi)*cos(chi-psi)/Vg*dt
        //f_mm[4] = 0*dt
        //f_mm[5] = 0*dt
        f_mm[6] = psi_dot*dt


        return f_mm
    }


    fun generateQd_mmMotionModel( dt: Double,
                              airspeed: Double,
                              pitchrate: Double,
                              yawrate: Double,
                              roll: Double,
                              pitch: Double,
                              xhat: Matrix<Double>,
                              tau_vv: Double,
                              sigma_vv: Double): Matrix<Double> {






        //Calculate Qd of motion model
          //B=df(x)/du and Qd_mm = B*Q_ud*B^T
          //Where Qd_mm is the motion model Qd and Q_ud is the estimated noise on each input in u. This assumes Q_u*dt=Q_ud.


        //current states
        var Vg   = xhat[2]
        var chi  = xhat[3]
        var Wn   = xhat[4]
        var We   = xhat[5]
        var psi  = xhat[6]
        var Q_ud = eye(5)
        val g    = 9.81 //Gravity m/s

        //Inputs
        var Va = airspeed
        var q = pitchrate
        var r = yawrate
        var phi = roll
        var theta = pitch

        //Set noise on inputs
        Q_ud[0,0] = 1*dt //Noise on Va airspeed (m/s)
        Q_ud[1,1] = (.1*Math.PI/180)*dt //Noise on q pitch ang rate (rads)
        Q_ud[2,2] = (.1*Math.PI/180)*dt //Noise on q pitch ang rate (rads)
        Q_ud[3,3] = (1*Math.PI/180)*dt //Noise on phi aircraft roll (rads)
        Q_ud[4,4] = (1*Math.PI/180)*dt //Noise on theta aircraft pitch (rads)


        //Calculate B where B=df(x)/du
        var eq_1=We*cos(psi)-Wn*sin(psi)
        var eq_2=r*cos(phi)+q*sin(phi)
        var eq_3=q*cos(phi)-r*sin(phi)
        var eq_4=Vg*cos(theta)

        var B=zeros(7,5)

        B[2,0..4] = mat[eq_1*eq_2/eq_4, Va*sin(phi)*eq_1/eq_4, Va*cos(phi)*eq_1/eq_4, Va*eq_1*eq_3/eq_4,Va*sin(theta)*eq_1*eq_2/(Vg*cos(theta)*cos(theta))]
        B[3,3]    = g*cos(chi-psi)/(Vg*cos(phi)*cos(phi))
        B[6,0..4] = mat[0,sin(phi)/cos(theta), cos(phi)/cos(theta), eq_3/cos(theta), sin(theta)*eq_2/(cos(theta)*cos(theta))]

        //calculate Qd_mm
        var Qd_mm = B*Q_ud*B.T

        //add q for all states
        return Qd_mm
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
        val F = zeros(9, 9)
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