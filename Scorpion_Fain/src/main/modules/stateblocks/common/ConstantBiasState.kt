package modules.stateblocks.common

import golem.*
import golem.containers.*
import golem.matrix.*
import scorpion.filters.sensor.containers.Dynamics
import scorpion.filters.sensor.containers.StateBlock

/**
 * Implements a 1x1 bias state modeled by
 *
 * xDot = 0
 *
 */
class ConstantBiasState : StateBlock {
    override var numStates: Int
    override var label: String

    /**
     *     Constructor takes a label, which identifies this block in the filter
     */
    constructor(label: String) {
        this.label = label
        this.numStates = 1
    }

    // Unused for this example, so throw an error if someone tries to use it
    override fun receiveAuxData(auxData: Any) {
        throw UnsupportedOperationException()
    }

    // This function takes in the current state estimate and time,
    // and produces the needed dynamics equation parameters (f, Phi, Qd)
    override fun generateDynamics(xhat: Matrix<Double>, timeFrom: Time, timeTo: Time): Dynamics {

        // Define the f(x) propagation function as f(x_(k-1)) = 1 * x_(k-1)
        fun f(x: Matrix<Double>): Matrix<Double> {
            // Creates a 1x1 matrix containing x
            return x
        }

        var Phi = mat[1]
        var Qd = mat[0]

        return Dynamics(::f, Phi, Qd)
    }
}

