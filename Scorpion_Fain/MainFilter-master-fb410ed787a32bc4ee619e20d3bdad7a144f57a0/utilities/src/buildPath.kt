import golem.mat

/**
 * Created by daniel on 08/09/16.
 */


fun buildPathSVOExample (iteration: Int):String {
    //initialize path strings
    val beginning = "/home/daniel/workspace/Datasets/sin2_tex2_h1_v8_d/img/frame_"
    val end = "_0.png"
    var middle = Integer.toString(iteration+2)
    val dim = middle.length
    for (j in 0..6 - dim - 1) {
        middle = "0" + middle
    }
    var path = beginning + middle + end
    return path
}

fun buildPathASPN1 (iteration: Int):String {
    //initialize path strings
    //val beginning = "/mnt/hgfs/Shared Folder/ASPNFlightForCarson/Data/GC1350_edited/1662_58"
    //val beginning = "/mnt/hgfs/Shared Data/DataWithout4000/GC1350_edited/1662_58"
    val beginning = "/mnt/hgfs/Shared Data/CarsonSet3/GC1350_edited/1662_5"
    val end = ".tiff"
    val decimal = (iteration.toDouble().mod(5)/5).toString()
    var middle = ""
    val integer = ((iteration)/5 + 78614).toString()
    val middleLength: Int
    if(decimal != "0.0") {
        middle = integer+decimal.substring(1)
        middleLength = 7
    }
    else {
        middle = integer
        middleLength = 5
    }
    while(middle.length<middleLength) {
        middle = "0"+middle
    }

    var path = beginning + middle + end
    return path
}

fun buildPathASPN2 (iteration: Int):String {
    //initialize path strings
    val beginning = "/mnt/hgfs/Shared Folder/ASPNFlightForCarson/Data/GE4000_edited/1662_58"
    val end = ".tiff"
    val decimal = (iteration.toDouble().mod(5)/5).toString()
    var middle = ""
    val integer = (iteration/5 + 2961).toString()
    if(decimal != "0.0") {
        middle = integer+decimal.substring(1)
    }
    else {
        middle = integer
    }

    var path = beginning + middle + end
    return path
}
