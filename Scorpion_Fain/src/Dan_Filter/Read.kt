import golem.matrix.Matrix
import golem.zeros
import java.io.BufferedReader
import java.io.FileReader
import java.util.*

/**
 * Created by ucav on 9/9/2016.
 */

fun readImus(fileNameL:String, fileNameR:String, DELIMITER:String): ArrayList<Matrix<Double>> {
    var fileReaderL = BufferedReader(FileReader(fileNameL))
    var lineL=fileReaderL.readLine()
    var tokensL=lineL.split(DELIMITER)

    var fileReaderR = BufferedReader(FileReader(fileNameR))
    var lineR=fileReaderR.readLine()
    var tokensR=lineR.split(DELIMITER)

    var data: ArrayList<Matrix<Double>> = ArrayList(160 * 100)

    var row = zeros(14)
    for (i in 0..tokensL.size-1)
    {
        row[i]=tokensL[i].toDouble()
    }
    for (i in 0..tokensR.size-1)
    {
        row[i+7]=tokensR[i].toDouble()
    }
    data.add(row)

    while( { lineL = fileReaderL.readLine() ; lineL}()  !=null&&{ lineR = fileReaderR.readLine() ; lineR}()  !=null )
    {
        tokensR=lineR.split(DELIMITER)
        tokensL=lineL.split(DELIMITER)
        row = zeros(14)
        for (i in 0..tokensL.size-1)
        {
            row[i]=tokensL[i].toDouble()
        }
        for (i in 0..tokensR.size-1)
        {
            row[i+7]=tokensR[i].toDouble()
        }
        data.add(row)
    }
    //removing extra data row so that the truth and mechanized values have the same number of entries
    var testing=data.size
    data.removeAt(testing-1)
    //data.removeAt(0)
    return data
}


//Written by Capt Daniel Johnson
fun readTdf(fileName:String, DELIMITER:String, header:Boolean): ArrayList<Matrix<Double>> {
    var fileReader = BufferedReader(FileReader(fileName))
    var line = fileReader.readLine()
    var tokens = line.split(DELIMITER)

    var data: ArrayList<Matrix<Double>> = ArrayList()

    var row = zeros(tokens.size)
    //var data = row

    //If there is no header do the first line, otherwise skip
    if (header==false)
    {
        for (i in 0..tokens.size - 1) {
            row[i] = tokens[i].toDouble()
        }

        //var data=row
        data.add(row)
    }

    // This section is from when I built the matrix instead of the list
    /*else
    {
        line = fileReader.readLine()
        tokens = line.split(DELIMITER)
        for (i in 0..tokens.size - 1) {
            row[i] = tokens[i].toDouble()
        }
        var data=row
    }*/


    while ( { line = fileReader.readLine() ; line }() != null) {
        row = zeros(tokens.size)
        tokens = line.split(DELIMITER)
        for (i in 0..tokens.size - 1) {
            row[i] = tokens[i].toDouble()
        }

        data.add(row)
    }
    return data
}

//Written by Capt Daniel Johnson
fun convertArrayListToArray(list:ArrayList<Matrix<Double>>):Matrix<Double>{

    var rows=list.size
    var cols=list.get(0).numCols()
    var b=zeros(rows,cols)
    for(i in 0..rows-1)
    {
        var row = list[i]
        for(j in 0..cols-1)
        {
            b[i,j]=row.get(j)
        }
    }
    //   System.out.print(temp)
    return b
}