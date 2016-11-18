package modules.aux_data

/**
 * Aux data to encode contents of PosVel measurements (usually from FLY).
 *
 * @param contents: String denoting what the Measurement contains,
 * in the case when the file has some measurement value marked as
 * invalid. Should be "Position", "Velocity" or "Both".
 */
data class PosVelAuxData(val contents: PosVelMeasurementTypes){

    /**
     * Denotes allowable types.
     */
    enum class PosVelMeasurementTypes{POSITION, VELOCITY, BOTH}
}