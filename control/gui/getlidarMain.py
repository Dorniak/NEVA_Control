import sys
import struct
import math
import numpy as np
import datetime
import matplotlib.pyplot as plt
import math
import time
import queue
import threading
import multiprocessing
from mpl_toolkits import mplot3d
from matplotlib.animation import FuncAnimation
import sqlite3
import pandas as pd

sp = sys.platform
if (sp =='win'):
    import winsound as ws

#Devuelve el incremento min. de azimut dependiendo de la velocidad de rot. del LiDAR
def getDeltaAz(lidS):
    return(lidS/1500.)

def my_hex_unpack1(myhexdata):
    lenhexdata = len(myhexdata)
    #print(lenhexdata)
    myres = 0
    for i in np.arange(lenhexdata):
        myres = myres + int(str(myhexdata[i]),10)*np.power(256,lenhexdata-i-1)
    return(myres)

def time_offset1(firing, block):
    return((block*2 + firing//16)*55.5 + 2.8 *(firing%16))

#Func. de lectura propiamente dicha
#def getAll(lidConf, M, R, conn, lidarSpeed=600, pub="y"):
def getAll(lidConf, M, R, conn, pub="y"):
    #Obtenemos los datos necesarios para la ejec. del programa. Por defecto se leen todos 
    altMat1 = lidConf.getAltMat1()
    t_offset =  lidConf.get_t_offset()
    lidarSpeed = lidConf.getLidarSpeed()
    sock = lidConf.getMsopSocket()
    securityDistance = lidConf.getSecDis()
    
    #Calculamos el inc. min. de azimut, esta vez dentro de la func.
    #A su vez, podemos calcular, otra vez y dentro de la func. los tam de las matrices, sin leerlas
    deltaA = getDeltaAz(lidarSpeed)
    msize = math.floor(360/deltaA)
    
   
    
    # M = np.zeros((17,msize,2))
    # R = np.zeros((17,msize,3))
    
    # print (M)
    # print(R) 
    
    #Second firing
    #M1 = np.zeros((17,msize,2))
    #R1 = np.zeros((17,msize,3))
    
    #Seleccionamos ind. en func del azimut.
    #indexAzMin = math.floor(azmlmin/deltaA)
    #indexAzMax = math.floor(azmlmax/deltaA)
    
    
    #La matriz de los ang. de alturas viene en una forma poco conveniente para filtrarla luego. Hay que dar la vuelta a la submatriz inferior
    altMatRev = altMat1[8:,:][::-1]
    altMat1[8:,:] = altMatRev
    altA = altMat1[:,0]

    #De este modo sabemos loos ind. max. y min. que hay que considerar.
    #filtered_lst = [(x,y) for x,y in enumerate(altA) if ((y >= altmin) & ( y <= altmax ))]
    # print(altA)
    #Extraemos estos ind.
    #indexAlMax, _ = max(filtered_lst)
    #indexAlMin, _ = min(filtered_lst) 
    
    #print("Index Alt Min: %s, Index Alt Max: %s, Alt List: %s" % (indexAlMin, indexAlMax, filtered_lst))
    #Necesitamos los ind. de los datos de cada mensaje. Se leen de golpe tanto por bloques (12) como por datos (16 * 2 disparos) por bloque 

    #Indexes for structures
    blockAxis = np.arange(0,1200,100)
    dataAxis = np.arange(46,46+48,3)
    #Second firing
    #dataAxis1 = np.arange(94,94+48,3)

    #Indexes for data
    indexD = [ dataA + blockA for dataA in dataAxis for blockA in blockAxis]
    #Second firing
    #indexD1 = [ dataA + blockA for dataA in dataAxis1 for blockA in blockAxis]

    #Leemos los indicadores de bloque, para segurarnos que leemos correctamente.

    #Indexes for block indicatorstruct.unpack(">H", bytearray(data[i:i+2]))[0], hex(struct.unpack(">H", bytearray(data[i:i+2]))[0])))
    
    #E igualmente para los azimut (por ahora nos quedamos con los ind.)

    #Indexes for block azimuths
    indAz = np.arange(44, 1244, 100)
    #for i in indAz:
        #print("Azimuth: %s, Azimuth hex: %s\n" % (struct.unpack(">H", bytearray(data[i:i+2]))[0], hex(struct.unpack(">H", bytearray(data[i:i+2]))[0])))

    #N. de iteraciones necesarias para rellenar toda la matriz
    niter = int(msize / 12)
    #Resol. de dist.       
    resoL = 0.5e-2

    #while(True):
    for i in range(niter):
        #Leemos un paquete MSOP (datos)
        data, addrMSOP = sock.recvfrom(1290)
        #Decodificamos el header (cabecera) para estar seguros de que estamos decodificando bien el paquete.
        header = data[0:8]

        #for i in range(8):
        #    print(header[i], hex(header[i]))

        #Decodificamos el time stamp del mensaje
    
        timestamp = data[20:31]

        year_m = int(str(timestamp[0]),10)
        month_m = int(str(timestamp[1]),10)
        day_m = int(str(timestamp[2]),10)
        hour_m = int(str(timestamp[3]),10)
        minute_m = int(str(timestamp[4]),10)
        second_m = int(str(timestamp[5]),10)
        millisecond_m = struct.unpack(">H", bytearray(timestamp[6:8]))[0]
        microsecond_m = struct.unpack(">H", bytearray(timestamp[8:10]))[0]


        #print("Year: %s, Month: %s, Day: %s, Hour: %s, Minute: %s, Second: %s, Millisecond: %s, Microsecond: %s" % (year_m, month_m, day_m, hour_m, minute_m, second_m, millisecond_m, microsecond_m))  

        #Obtenemos el modelo de LiDAR

        lidarmodel = int(str(data[30]))

        #print ("LiDAR model: %s, Hex: %s" % (lidarmodel, hex(lidarmodel)))    

        #Ahora leemos los azimut, cuyos ind. ya obtuvimos anteriormente.

        azList = [struct.unpack(">H", bytearray(data[i:i+2]))[0]/100. for i in indAz]
        #Second firing
        #azList1 = [struct.unpack(">H", bytearray(data[i:i+2]))[0]/100. + (deltaA/2.)/100. for i in indAz]

        #Asignamos los in en func. del azimut.
    
        indexA = [math.floor(i/deltaA) for i in azList]
        #Second firingh
        #indexA1 = [math.floor(i/deltaA) for i in azList1]
       #print(indexA)
        #Second firing
        #print(indexA1)
    
        #if (indexF==0):    
        #    azList = [struct.unpack(">H", bytearray(data[i:i+2]))[0] for i in indAz]    
        #    for i in range(11):
        #        if (azList[i]>azList[i+1]):
        #            azCorr[i+1] = azCorr[i+1] + 36000
        #            xx = xx + 1
        #        else:
        #            azCorr[i] = azList[i]
        #    azCorr = [struct.unpack(">H", bytearray(data[i:i+2]))[0] for i in indAz]
        #else:
        #    azList = [struct.unpack(">H", bytearray(data[i:i+2]))[0] for i in indAz]
    
        #print((azList/100.)/0.4)    
    
        #print("Azimut: %s\t, Delta Azimut: %s" % (azList, np.median(np.diff(np.array(azList))))) 

        #Decodificamos distancias
        dist_b = [struct.unpack(">H", bytearray(data[i:i+2]))[0] for i in indexD]
        #Second firing
        #dist_b1 = [struct.unpack(">H", bytearray(data[i:i+2]))[0] for i in indexD1]

        #Decodificamos reflectividades
        refl_b = [int(str(data[i+2]),10) for i in indexD]
        #Second firing
        #refl_b1 = [int(str(data[i+2]),10) for i in indexD1]
        ##refl_b = [data[i+2] for i in indexD]

        #Asignamos forma matricial a las distancias, teniendo en cuenta las modificaciones en la mat. de ang. de alt.
        distM = np.reshape(dist_b, (16,12))
        distRev = distM[8:,:][::-1]
        distM[8:,:] = distRev 
        #Second firing
        #distM1 = np.reshape(dist_b1, (16,12))
        #distRev1 = distM1[8:,:][::-1]
        #distM1[8:,:] = distRev1

        #Lo mismo para las reflectividades.
        reflM = np.reshape(refl_b, (16,12))
        reflRev = reflM[8:,:][::-1]
        reflM[8:,:] = reflRev 
        #Second firing
        #reflM1 = np.reshape(refl_b1, (16,12))
        #reflRev1 = reflM1[8:,:][::-1]
        #reflM1[8:,:] = reflRev1

        #Rellenamos las matrices M y R
        M[0,indexA,0] = azList
        #Second firing
        #M1[0,indexA1,0] = azList1
        M[1:17,indexA,0] = distM * resoL
        #Second firing
        #M1[1:17,indexA1,0] = distM1 * resoL
        M[0,indexA,1] = azList
        #Second firing
        #M1[0,indexA1,1] = azList1
        
        M[1:17,indexA,1] = reflM
        #Second firing
        #M1[1:17,indexA1,1] = reflM1
        R[0,indexA,0] = azList
        R[0,indexA,1] = azList
        R[0,indexA,2] = azList
        #Second firing
        #R1[0,indexA1,0] = azList1
        #R1[0,indexA1,1] = azList1
        #R1[0,indexA1,2] = azList1

        #R[0,indexA] = azList
        #R[1:17,indexA] = reflM
    
        #print(distM)
        ##print(dist_b)
        #print(reflM)
        ##print(refl_b)
    
        #print(M)

    #Matrices de angs de altura y azimut para la transformac. en cartesianas

    alM = np.repeat(altMat1,niter,1).reshape(16,msize)
    azM = np.repeat(R[0,:,0],16,0).reshape(msize,16).T
    #Second firing
    #azM1 = np.repeat(R1[0,:,0],16,0).reshape(900,16).T

    #Por si queremos guardar los resultados

    filename1 = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")

    #np.savetxt('AltMat_' + filename1 + '_3alt.txt', alM)
    #np.savetxt('AziMat_' + filename1 + '_3azi0.txt', azM)
    #Second firing
    #np.savetxt('AziMat_' + filename1 + '_3azi1.txt', azM1)
    #print(np.min(M[1:,:,0] ))

    #Matrices en cartesianas

    R[1:,:,0] = M[1:,:,0] * np.cos(np.deg2rad(alM)) * np.sin(np.deg2rad(azM))
    R[1:,:,1] = M[1:,:,0] * np.cos(np.deg2rad(alM)) * np.cos(np.deg2rad(azM))
    R[1:,:,2] = M[1:,:,0] * np.sin(np.deg2rad(alM))
    
    #Medimos los valores nulos y su proporc.
    sizeArr = M[1:,:,0].size
    numberofzeroes = np.count_nonzero(M[1:,:,0] == 0.0)
    ratiozeroes = numberofzeroes / sizeArr 
    print("Zero values: %s, Ratio zeros = %s\n" % (numberofzeroes, ratiozeroes))
    
    if ((pub=="y") | (pub=="s")):
        #PUblicamos en memoria mediante sqlite3
        num_rows, num_cols = R[:, :, 0].shape

        Rdf0 = pd.DataFrame(data=R[:, :, 0], index=np.arange(num_rows), columns=np.arange(num_cols))
        Rdf1 = pd.DataFrame(data=R[:, :, 1], index=np.arange(num_rows), columns=np.arange(num_cols))
        Rdf2 = pd.DataFrame(data=R[:, :, 2], index=np.arange(num_rows), columns=np.arange(num_cols))

        Mdf0 = pd.DataFrame(data=M[:, :, 0], index=np.arange(num_rows), columns=np.arange(num_cols))
        Mdf1 = pd.DataFrame(data=M[:, :, 1], index=np.arange(num_rows), columns=np.arange(num_cols))

        Rdf0.to_sql("R0", conn, if_exists="replace")
        Rdf1.to_sql("R1", conn, if_exists="replace")
        Rdf2.to_sql("R2", conn, if_exists="replace")
        Mdf0.to_sql("M0", conn, if_exists="replace")
        Mdf1.to_sql("M1", conn, if_exists="replace")
    else:
        pass


    return(R,M)