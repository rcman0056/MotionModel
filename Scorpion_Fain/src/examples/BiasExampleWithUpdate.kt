import golem.*
import golem.containers.*
import modules.measurements.DirectMeasurementProcessor
import modules.stateblocks.common.ConstantBiasState
import scorpion.buffers.Buffer
import scorpion.filters.sensor.StandardSensorEKF
import scorpion.filters.sensor.containers.Measurement

object BiasExampleWithUpdate {
    @JvmStatic
    fun main(args: Array<String>) {
        // Create our state block, measurement processor, and filter instance
        val block = ConstantBiasState(label = "myblock")
        val processor = DirectMeasurementProcessor(label = "myprocessor",
                                                   HValue = 2.0,
                                                   blockLabel = "myblock")
        val filter = StandardSensorEKF(curTime = Time(0.0), buffer = Buffer(standard = TimeStandard.WALL))

        // Add the block and processor to the filter
        filter.addStateBlock(block)
        filter.addMeasurementProcessor(processor)

        // Set initial covariance
        filter.setStateBlockCovariance(label = "myblock",
                                       covariance = mat[1])

        // Propagate to 0.1 seconds
        filter.propagate(time = Time(0.1))

        // Make a measurement at 0.1 seconds stating that z=0 with cov 1.001
        val rawMeas = Measurement(processorLabel = "myprocessor",
                                  timeReceived = Time(0.1),
                                  timeValidity = Time(0.1),
                                  measurementData = mat[0],
                                  auxData = null,
                                  measurementCov = mat[1.001])

        // Update the filter's estimate with the measurement
        filter.update(measurement = rawMeas)

        // Get the estimate after the update
        var out = filter.getStateBlockEstimate("myblock")
        var outCov = filter.getStateBlockCovariance("myblock")
    }

}