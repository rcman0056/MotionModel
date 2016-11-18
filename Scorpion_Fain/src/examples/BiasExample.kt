import golem.containers.*
import modules.stateblocks.common.ConstantBiasState
import scorpion.filters.sensor.StandardSensorEKF

/**
 *  An simple example of setting up a filter with a single bias state.
 */
object BiasExample {
    @JvmStatic
    fun main(args: Array<String>) {
        val block = ConstantBiasState("mybias")
        val filter = StandardSensorEKF()

        filter.addStateBlock(block)
        filter.propagate(Time(0.1, TimeStandard.GPS))
    }
}
