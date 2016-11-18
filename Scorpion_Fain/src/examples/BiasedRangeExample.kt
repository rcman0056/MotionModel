import golem.*
import golem.containers.*
import modules.measurements.BiasedRangeProcessor
import modules.stateblocks.common.ConstantBiasState
import modules.stateblocks.common.FOGMPos3
import navutils.containers.Vector3
import scorpion.filters.sensor.StandardSensorEKF
import scorpion.filters.sensor.containers.Measurement


object BiasedRangeExample {
    @JvmStatic
    fun main(args: Array<String>) {
        val ekf = StandardSensorEKF()
        val biasBlock = ConstantBiasState(label = "radarblock1")
        val posBlock = FOGMPos3(label = "pos1", tau = 1.0, Q = 1.0)
        val rangeProcessor = BiasedRangeProcessor(label = "radarprocessor1",
                                                  biasLabel = "radarblock1",
                                                  posLabel = "pos1",
                                                  beaconLocation = Vector3(1.0, 1.0, 1.0))

        ekf.addStateBlock(biasBlock)
        ekf.addStateBlock(posBlock)
        ekf.addMeasurementProcessor(rangeProcessor)

        val times = mat[1, 2, 3]
        val measurements = mat[1.1, 2.2, 3.3]
        measurements.forEachIndexed { timeIdx, meas ->
            val measObj = Measurement("radarprocessor1",
                                      Time(times[timeIdx], TimeStandard.GPS),
                                      Time(times[timeIdx], TimeStandard.GPS),
                                      mat[3.456],
                                      null,
                                      mat[0.0])
            ekf.propagate(Time(times[timeIdx], TimeStandard.GPS)) // Optional
            ekf.update(measObj)
        }

    }

}
