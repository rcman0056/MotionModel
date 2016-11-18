package modules.stateblocks.common

import golem.*
import golem.matrix.*
import navutils.calcVanLoan
import scorpion.filters.sensor.containers.StateBlock

/**
 * Class that allows for discretization of continuous time matrices.
 */
abstract class DiscreteStateBlock : StateBlock {

    var Phi: Matrix<Double>
    var Qd: Matrix<Double>

    constructor(numStates: Int, label: String, Phi: Matrix<Double>, Qd: Matrix<Double>) {
        this.Phi = Phi
        this.Qd = Qd
        this.numStates = numStates
        this.label = label
    }

    /**
     * Generates Phi and Qd to propagate a this set of states over dt.
     *
     * @param F: Linearized propagation matrix.
     * @param Q: Continuous time process noise matrix.
     * @param dt: Delta time over which F and Q are valid.
     * @param order: Discretization level to use. 1 is first order, 2 is
     * second, everything else defaults to full.
     */
    fun discretize(F: Matrix<Double>, G: Matrix<Double>, Q: Matrix<Double>,
                   dt: Double, order: Int = 0) {
        if (order == 1) {
            Phi = eye(F.numCols()) + F * dt
            Qd = G * Q * G.T * dt
        } else if (order == 2) {
            Phi = eye(F.numCols()) + F * dt + pow(F * dt, 2)
            var qMap = G * Q * G.T
            Qd = (Phi * qMap * Phi.T + qMap) * dt / 2.0
        } else {
            Phi = expm(F * dt)
            Qd = calcVanLoan(F, G, Q, dt)
        }
    }
}

