package modules.stateblocks.common

import golem.*
import golem.containers.*
import golem.matrix.*
import scorpion.filters.sensor.containers.Dynamics
import scorpion.filters.sensor.containers.StateBlock

/**
 * An implementation of StateBlock that represents a first-order Gauss-Markov stochastic process
 * in three dimensions (i.e. FOGM position).
 *
 * @param label The label uniquely identifying this particular set of states.
 * @param tau The FOGM time constant.
 * @param Q The discrete-time FOGM noise source
 * @param t0 The time to initialize this block to.
 */
class FOGMPos3 : StateBlock {
    override var numStates: Int
    override var label: String

    var tau: Double
    var Q: Double

    constructor(label: String,
                tau: Double,
                Q: Double) {
        this.label = label
        this.numStates = 3
        this.tau = tau
        this.Q = Q
    }

    override fun receiveAuxData(auxData: Any) {
        throw UnsupportedOperationException()
    }

    override fun generateDynamics(xhat: Matrix<Double>, timeFrom: Time, timeTo: Time): Dynamics {
        var dt = timeTo.time - timeFrom.time

        var Phi = eye(3) * (E pow (-dt / tau))
        var Qd = eye(3) * Q
        fun f(x: Matrix<Double>): Matrix<Double> {
            return Phi * x
        }

        return Dynamics(::f, Phi, Qd)
    }
}