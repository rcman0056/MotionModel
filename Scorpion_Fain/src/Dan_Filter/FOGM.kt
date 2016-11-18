/**
 * Created by ucav on 11/2/2016.
 */

data class FOGM(var sigma:Double, var timeconstant:Double)

fun FOGMParameterLookupX(range: Double):FOGM
{
    var sigma=0.0
    var timeconstant=0.1

    if(range>100) {
        sigma =.04*(range)-2
        timeconstant=2.0
    }
    else if(range>50)
    {
        sigma=.036*(range)-1.6
        //sigma=.03*(range)-1.0
        timeconstant=5.0
    }
    else
    {
        sigma=.2
        timeconstant=5.0
    }
    return FOGM(sigma,timeconstant)
}

fun FOGMParameterLookupY(range: Double):FOGM
{
    var sigma=0.0
    var timeconstant=0.1

    if(range>100) {
        sigma =.04*(range)-2
        timeconstant=2.0
    }
    else if(range>50)
    {
        sigma=.038*(range)-1.8
        //sigma=.03*(range)-1.0
        timeconstant=5.0
    }
    else
    {
        sigma=.1
        timeconstant=50.0
    }
    return FOGM(sigma,timeconstant)
}

fun FOGMParameterLookupZ(range: Double):FOGM
{
    var sigma=0.0
    var timeconstant=0.1

    if(range>100) {
        sigma =.04*(range)-2
        timeconstant=2.0
    }
    else if(range>50)
    {
        sigma=.036*(range)-1.6
        //sigma=.03*(range)-1.0
        timeconstant=5.0
    }
    else
    {
        sigma=.1
        timeconstant=50.0
    }
    return FOGM(sigma,timeconstant)
}