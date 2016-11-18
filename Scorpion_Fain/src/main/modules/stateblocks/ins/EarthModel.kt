package modules.stateblocks.ins

import golem.*
import golem.matrix.*

/**
 * An interface specifying a model for the Earths gravity.
 * @param earth An instance of the EarthModel for the current position
 * @param alt_msl The current altitude, mean sea level (m)
 */
interface GravityModel {
    fun calculateGravity(earth: EarthModel, alt_msl: Double): Double
}

class GravityModelTittertonAndWeston : GravityModel {
    /**
     *     Determine the Earth's local gravity magnitude.
     *     @param earth Earth model object.
     *     @param alt_msl MSL Altitude (m).
     *
     *     @return Gravity magnitude (m/s*s).
     *
     */
    override fun calculateGravity(earth: EarthModel, alt_msl: Double): Double {
        val g0 = 9.780318 * (1 + 5.3024e-3 * pow(earth.sin_l, 2) - 5.9e-6 * pow(earth.sin_2l, 2))
        val r = 1 + alt_msl / earth.r_zero
        if (alt_msl >= 0.0)
            return g0 / pow(r, 2)
        else
            return g0 * r
    }
}

/**
 * A model for calculating gravity near-Earth based on Schwartz.
 */
class GravityModelSchwartz : GravityModel {
    val A1 = 9.7803267715
    val A2 = 0.0052790414
    val A3 = 0.0000232718
    val A4 = -3.0876910891e-6
    val A5 = 4.3977311e-9
    val A6 = 7.211e-13

    /**
     * Determine the Earth's local gravity magnitude.
     * @param earth Earth model object.
     * @param alt_msl MSL Altitude (m).
     * @return Gravity magnitude (m/s*s).
     */
    override fun calculateGravity(earth: EarthModel, alt_msl: Double): Double {
        val sin2_l = pow(earth.sin_l, 2)
        val sin4_l = pow(sin2_l, 2)

        return A1 * (1.0 + A2 * sin2_l + A3 * sin4_l) + (A4 + A5 * sin2_l) * alt_msl + A6 * pow(alt_msl, 2)
    }
}

/**
 * WGS Approximation to the Earth's model.
 *
 * @param pos The current ellipsoidal position (rad, rad, m).
 * @param vel The current NED velocity (m/s, m/s, m/s).
 * @param gravity The gravity model to use for computation
 */
class EarthModel(pos: Matrix<Double>, vel: Matrix<Double>, gravity: GravityModel = GravityModelSchwartz()) {
    companion object {
        val RAD_E = 6378137.0                       // Earth semi-major axis, (m)
        val OMEGA_E = 7.2921151467e-5               // Earth rate, (rad/s)
        val F = 1 / 298.257223563                   // Flattening of Earth
        val ECC_SQUARE = F * (2 - F)                // Eccentricity squared
        val ECC = sqrt(ECC_SQUARE)                  // Major eccentricity of the ellipsoid
    }

    val lat = pos[0]
    val alt_msl = pos[2]

    val v_n = vel[0]
    val v_e = vel[1]

    // Update Earth parameters.
    val sin_l = sin(lat)
    val cos_l = cos(lat)
    val tan_l = sin_l / cos_l
    val sec_l = 1 / cos_l
    val sin_2l = sin(2 * lat)

    // Take into account earth is elliptical...Equation 3.82, pg 54.
    val ell = 1.0 - ECC_SQUARE * pow(sin_l, 2)
    val r_n = RAD_E * (1.0 - ECC_SQUARE) / pow(ell, 1.5)
    val r_e = RAD_E / sqrt(ell)
    val r_zero = sqrt(r_n * r_e)

    // Used to convert degrees lat/lon into meters.
    val lat_factor = r_n + alt_msl
    val lon_factor = cos_l * (r_e + alt_msl)

    // Calculate transport rate, omega_en, in the nav frame.
    // NOTE: This is a rate!
    val omega_en_n = mat[v_e / (r_e + alt_msl), -v_n / (r_n + alt_msl), -v_e * tan_l / (r_e + alt_msl)]

    // Calculate earth rate, omega_ie, in the nav frame.
    // NOTE: This is a rate!
    val omega_ie_n = mat[OMEGA_E * cos_l, 0.0, -OMEGA_E * sin_l]

    // Calculate turn rate of navigation frame with respect to inertial frame, omega_in_n.
    // NOTE: This is a rate!
    val omega_in_n = omega_ie_n + omega_en_n

    val g_n = mat[0.0, 0.0, gravity.calculateGravity(this, alt_msl)]
}