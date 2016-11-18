package preprocessors

import scorpion.filters.sensor.containers.Measurement

/**
 * A common use case for scorpion is to process data generated
 * by code that isnt scorpion-aware. This class abstracts a
 * preprocessor which converts data from an arbitrary measurement
 * type into the [Measurement] type a scorpion filter needs.
 *
 * While it is possible to simply do any needed pre-processing outside
 * of the scorpion framework, this class allows us to offer a set of
 * off-the-shelf preprocessors for common data sources, such as ASPN
 * data. An implementation of this class should state which
 * [MeasurementProcessor] its output Measurement should be compatible with.
 * A typical processing pipeline will look like:
 *
 */
interface Preprocessor<T> {
    fun process(rawMeas: T):Measurement
}