package modules.stateblocks.common

import golem.*
import golem.containers.*
import golem.matrix.*
import scorpion.filters.sensor.containers.Dynamics

/**
 * State block that is used to estimate a time-varying FOGM state.
 *
 * @param label: Label used to refer to this StateBlock within a filter.
 * @param biasSigma: 1-sigma value describing the uncertainty of the state (m).
 * @param timeConstant: Decay term for state (s).
 */
class FOGMState(override var label: String,
                val biasSigma: Double,
                val timeConstant: Double) : DiscreteStateBlock(1, label, mat[1], mat[1]) {


    override var numStates = 1

    /**
     * Unused for this function.
     */
    override fun receiveAuxData(auxData: Any) {
        throw UnsupportedOperationException()
    }

    /**
     * Propagates a time-varying FOGM state forward in time.
     *
     * @param xhat: Current state estimate vector.
     * @param timeFrom: Time at which xhat is valid.
     * @param timeTo: Time to propagate to. Must be with respect to the same
     * reference as timeFrom.
     *
     * @return: Dynamics object that will propagate xhat to timeTo.
     */
    override fun generateDynamics(xhat: Matrix<Double>, timeFrom: Time, timeTo: Time): Dynamics {
        var F = mat[-1 / timeConstant]
        var Q = mat[2.0 * pow(biasSigma, 2) / timeConstant]
        val dt = timeTo.time - timeFrom.time
        var f = { x: Matrix<Double> -> Phi * x }
        this.discretize(F, eye(Q.numRows()), Q, dt)
        return Dynamics(f, Phi, Qd)
    }
}