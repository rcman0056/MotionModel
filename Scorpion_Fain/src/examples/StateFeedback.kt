import golem.*
import golem.containers.*
import golem.matrix.*
import modules.stateblocks.ins.getImuModelHG1700
import scorpion.filters.sensor.StandardSensorEKF
import scorpion.filters.sensor.containers.Dynamics
import scorpion.filters.sensor.containers.StateBlock


/**
 *  A simple example showing state feedback operating properly.
 */
object StateFeedback {
    @JvmStatic
    fun main(args: Array<String>) {
        var model = getImuModelHG1700()

        val block = object : StateBlock {

            override var numStates = 2

            override var label = "posvel"

            override fun receiveAuxData(auxData: Any) {
                throw UnsupportedOperationException()
            }

            //@formatter:off

            override fun generateDynamics(xhat: Matrix<Double>, timeFrom: Time, timeTo: Time): Dynamics {

                val dt = 1.0
                val F = mat[0, 1 end
                            0, -.5]
                val Qd = eye(2) * .1
                val Phi = expm(F * dt)
                val f = { it: Matrix<Double> -> Phi * it }

                val out = Dynamics(f, Phi, Qd)
                return out
            }
            //@formatter:on

        }

        val filter = StandardSensorEKF()
        filter.addStateBlock(block)


    }
}