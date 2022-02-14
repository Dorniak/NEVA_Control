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
import sys

sp = sys.platform
if (sp =='win'):
    import winsound as ws

#Devuelve el incremento min. de azimut dependiendo de la velocidad de rot. del LiDAR
def getDeltaAz(lidS):
    return(lidS/1500.)

#def filtByAngle(lidConf, M, R, conn, azmlmin=0., azmlmax=360., altmin=-16., altmax=16., lidarSpeed=600, pub="y"):
def filtByAngle(lidConf, M, R, conn, azmlmin=0., azmlmax=360., altmin=-16., altmax=16., pub="y"):
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
    indexAzMin = math.floor(azmlmin/deltaA)
    indexAzMax = math.floor(azmlmax/deltaA)
    #La matriz de los ang. de alturas viene en una forma poco conveniente para filtrarla luego. Hay que dar la vuelta a la submatriz inferior
    altMatRev = altMat1[8:,:][::-1]
    altMat1[8:,:] = altMatRev
    altA = altMat1[:,0]
    #De este modo sabemos loos ind. max. y min. que hay que considerar.
    filtered_lst = [(x,y) for x,y in enumerate(altA) if ((y >= altmin) & ( y <= altmax ))]
    # print(altA)
    #Extraemos estos ind.
    indexAlMax, _ = max(filtered_lst)
    indexAlMin, _ = min(filtered_lst) 
    
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
    #
    # Second firing
    #R1[1:,:,0] = M1[1:,:,0] * np.cos(np.deg2rad(alM)) * np.sin(np.deg2rad(azM1))
    #R1[1:,:,1] = M1[1:,:,0] * np.cos(np.deg2rad(alM)) * np.cos(np.deg2rad(azM1))
    #R1[1:,:,2] = M1[1:,:,0] * np.sin(np.deg2rad(alM))


    #print(R)
    # np.savetxt('Distmatrix_' + filename1 + '_3d0.txt', M[:,:,0])
    #np.savetxt('Reflmatrix_' + filename1 + '_3r0.txt', M[:,:,1])
    #Second firing
    #np.savetxt('Distmatrix_' + filename1 + '_3d1.txt', M1[:,:,0])
    #np.savetxt('Reflmatrix_' + filename1 + '_3r1.txt', M1[:,:,1])

    #np.savetxt('Xmatrix_' + filename1 + '_3x0.txt', R[:,:,0])
    #np.savetxt('Ymatrix_' + filename1 + '_3y0.txt', R[:,:,1])
    #np.savetxt('Zmatrix_' + filename1 + '_3z0.txt', R[:,:,2])

    #Second firing
    #np.savetxt('Xmatrix_' + filename1 + '_3x1.txt', R1[:,:,0])
    #np.savetxt('Ymatrix_' + filename1 + '_3y1.txt', R1[:,:,1])
    #np.savetxt('Zmatrix_' + filename1 + '_3z1.txt', R1[:,:,2])

    #xl = R[1:,:,0][M[1:,:,0] != 65535 * resoL].flatten(order='F')
    #yl = R[1:,:,1][M[1:,:,0] != 65535 * resoL].flatten(order='F')
    #zl = R[1:,:,2][M[1:,:,0] != 65535 * resoL].flatten(order='F')
    #refL = M[1:,:,1][M[1:,:,0] != 65535 * resoL].flatten(order='F')

    #Filtramos lo que se encuentre en el "infinito" y lo que se encuentre pegado al LiDAR
    xl = R[1:,:,0][(M[1:,:,0] != 65535 * resoL) & (M[1:,:,0] != 0.0)].flatten(order='F')
    yl = R[1:,:,1][(M[1:,:,0] != 65535 * resoL) & (M[1:,:,0] != 0.0)].flatten(order='F')
    zl = R[1:,:,2][(M[1:,:,0] != 65535 * resoL) & (M[1:,:,0] != 0.0)].flatten(order='F')
    refL = M[1:,:,1][(M[1:,:,0] != 65535 * resoL) & (M[1:,:,0] != 0.0)].flatten(order='F')
#M[1:,:,0][M[1:,:,0] == 65535 * resoL] = 5.

    #xl = R[1:,:,0]
    #yl = R[1:,:,1]
    #zl = R[1:,:,2]
    #refL = M[1:,:,1]

    
    #Second firing
    #xl1 = R1[1:,:,0][M1[1:,:,0] != 65535 * resoL].flatten(order='F')
    #yl1 = R1[1:,:,1][M1[1:,:,0] != 65535 * resoL].flatten(order='F')
    #zl1 = R1[1:,:,2][M1[1:,:,0] != 65535 * resoL].flatten(order='F')
    #refL1 = M1[1:,:,1][M1[1:,:,0] != 65535 * resoL].flatten(order='F')

  # print('First Firing:\nMin(x): %s, Max(x): %s, Min(y): %s, Max(y): %s' % (np.min(xl),np.max(xl),np.min(yl),np.max(yl)))
    # Second firing
    # print('Second Firing:\nMin(x): %s, Max(x): %s, Min(y): %s, Max(y): %s' % (np.min(xl1),np.max(xl1),np.min(yl1),np.max(yl1)))
    xl1 = R[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),0][(M[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),0] != 65535 * resoL) & (M[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),0] != 0.0)].flatten(order='F')
    yl1 = R[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),1][(M[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),0] != 65535 * resoL) & (M[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),0] != 0.0)].flatten(order='F')
    zl1 = R[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),2][(M[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),0] != 65535 * resoL) & (M[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),0] != 0.0)].flatten(order='F')
    #zl1 = R[indexAlMin:(indexAlMax+1),indexAzMin:(indexAzMax+1),0][(M[indexAlMin:(indexAlMax+1),indexAzMin:(indexAzMax+1),0] != 65535 * resoL) & (M[indexAlMin:(indexAlMax+1),indexAzMin:(indexAzMax+1),0] != 0.0)].flatten(order='F')
    refL1 = M[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),1][(M[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),0] != 65535 * resoL) & (M[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),0] != 0.0)].flatten(order='F')
    #res = np.vstack((xl1,yl1,zl1,refL1))
    #res = res.reshape((4,int(res.size/4)))
    print(xl1.size)
    print(yl1.size)
    print(zl1.size)
    print(refL1.size)
    res = np.concatenate((xl1,yl1,zl1,refL1))
    res = res.reshape((4,int(res.size/4)))
    np.savetxt('res.txt',res) 
    dis = np.apply_along_axis(np.linalg.norm, 0, res[0:3,:])
    dis = dis.reshape((1,int(res.size/4)))
    #rdp1 = int(res1.size/4)
    
    if (np.min(dis) <= securityDistance):
        # Common way
        print('\a')
        #NO ADMIN PRIVILEGES (linux)/ PRIVILEGES IRRELEVANT (win)        
        if (sp=='linux'):
            print('SOS')
            #os.system("beep -f 555 -l 460")
            #os.system("cw -f SOS.txt")             
            #os.system("beep -f 3500 -l 250")   #Most unpleasant sound
        elif (sp =='win'):
            ws.Beep(3500,250) #Most unpleasant sound
    
    #global resultadoR
    #global resultadoM
    #global res
    #global dis

    resultadoR = R[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),:]
    resultadoM = M[(indexAlMin+1):(indexAlMax+2),indexAzMin:(indexAzMax+1),:]
    
    print(resultadoR.shape,resultadoM.shape)

    if ((pub=="y") | (pub=="s")):
        #Publicamos en memoria mediante sqlite3
        num_rows, num_cols = resultadoR[:, :, 0].shape
        resultadoRdf0 = pd.DataFrame(data=resultadoR[:, :, 0], index=np.arange(num_rows), columns=np.arange(num_cols))
        resultadoRdf1 = pd.DataFrame(data=resultadoR[:, :, 1], index=np.arange(num_rows), columns=np.arange(num_cols))
        resultadoRdf2 = pd.DataFrame(data=resultadoR[:, :, 2], index=np.arange(num_rows), columns=np.arange(num_cols))

        resultadoMdf0 = pd.DataFrame(data=resultadoM[:, :, 0], index=np.arange(num_rows), columns=np.arange(num_cols))
        resultadoMdf1 = pd.DataFrame(data=resultadoM[:, :, 1], index=np.arange(num_rows), columns=np.arange(num_cols))

        num_rows, num_cols = res.shape
        #print(res.T.shape)
        resdf = pd.DataFrame(data=res.T, index=np.arange(num_cols), columns=np.arange(num_rows))
        num_rows, num_cols = dis.shape
        #print(dis.T.shape)
        disdf = pd.DataFrame(data=dis.T, index=np.arange(num_cols), columns=np.arange(num_rows))
        resultadoRdf0.to_sql("resultadoR0", conn, if_exists="replace")
        resultadoRdf1.to_sql("resultadoR1", conn, if_exists="replace")
        resultadoRdf2.to_sql("resultadoR2", conn, if_exists="replace")
        resultadoMdf0.to_sql("resultadoM0", conn, if_exists="replace")
        resultadoMdf1.to_sql("resultadoM1", conn, if_exists="replace")
        resdf.to_sql("res", conn, if_exists="replace")
        disdf.to_sql("dis", conn, if_exists="replace")
    else:
        pass

    #return (R[indexAlMin:(indexAlMax+1),indexAzMin:(indexAzMax+1),:], M[indexAlMin:(indexAlMax+1),indexAzMin:(indexAzMax+1),:], res, dis) 
    return (resultadoR, resultadoM, res, dis) 
    #return rdp1

    
# return (R[indexAlMin:(indexAlMax+1),indexAzMin:(indexAzMax+1),:], M[indexAlMin:(indexAlMax+1),indexAzMin:(indexAzMax+1),:]) 
 
    
    
    
    
    #
    # ax = figlidar.add_ 
    # plt.cla()
    
    # Second firing
    # ax.scatter3D(xl1, yl1, zl1, c= refL1, cmap=plt.get_cmap('jet'))
    # ax.scatter3D(xl1, yl1, zl1, c= "red")
   
    # plt.pause(.01)
    # plt.ion()
    # plt.show()
    # plt.cla()
    # print("aaaaa")
    # plt.close('all') 

#def filtByArea(lidConf, M, R, conn, xmin1, xmax1, ymin1, ymax1, zmin1, zmax1, lidarSpeed=600, pub="y"):
def filtByArea(lidConf, M, R, conn, xmin1, xmax1, ymin1, ymax1, zmin1, zmax1, pub="y"):

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
    #
    # Second firing
    #R1[1:,:,0] = M1[1:,:,0] * np.cos(np.deg2rad(alM)) * np.sin(np.deg2rad(azM1))
    #R1[1:,:,1] = M1[1:,:,0] * np.cos(np.deg2rad(alM)) * np.cos(np.deg2rad(azM1))
    #R1[1:,:,2] = M1[1:,:,0] * np.sin(np.deg2rad(alM))


    #print(R)
    # np.savetxt('Distmatrix_' + filename1 + '_3d0.txt', M[:,:,0])
    #np.savetxt('Reflmatrix_' + filename1 + '_3r0.txt', M[:,:,1])
    #Second firing
    #np.savetxt('Distmatrix_' + filename1 + '_3d1.txt', M1[:,:,0])
    #np.savetxt('Reflmatrix_' + filename1 + '_3r1.txt', M1[:,:,1])

    #np.savetxt('Xmatrix_' + filename1 + '_3x0.txt', R[:,:,0])
    #np.savetxt('Ymatrix_' + filename1 + '_3y0.txt', R[:,:,1])
    #np.savetxt('Zmatrix_' + filename1 + '_3z0.txt', R[:,:,2])

    #Second firing
    #np.savetxt('Xmatrix_' + filename1 + '_3x1.txt', R1[:,:,0])
    #np.savetxt('Ymatrix_' + filename1 + '_3y1.txt', R1[:,:,1])
    #np.savetxt('Zmatrix_' + filename1 + '_3z1.txt', R1[:,:,2])

    #xl = R[1:,:,0][M[1:,:,0] != 65535 * resoL].flatten(order='F')
    #yl = R[1:,:,1][M[1:,:,0] != 65535 * resoL].flatten(order='F')
    #zl = R[1:,:,2][M[1:,:,0] != 65535 * resoL].flatten(order='F')
    #refL = M[1:,:,1][M[1:,:,0] != 65535 * resoL].flatten(order='F')

    #Filtramos lo que se encuentre en el "infinito" y lo que se encuentre pegado al LiDAR
    xl = R[1:,:,0][(M[1:,:,0] != 65535 * resoL) & (M[1:,:,0] != 0.0)].flatten(order='F')
    yl = R[1:,:,1][(M[1:,:,0] != 65535 * resoL) & (M[1:,:,0] != 0.0)].flatten(order='F')
    zl = R[1:,:,2][(M[1:,:,0] != 65535 * resoL) & (M[1:,:,0] != 0.0)].flatten(order='F')
    refL = M[1:,:,1][(M[1:,:,0] != 65535 * resoL) & (M[1:,:,0] != 0.0)].flatten(order='F')

    if ((xmin1 != 'NoLim') & (xmax1 != 'NoLim')):
        xl1 = xl[(xl >= xmin1) & (xl <= xmax1)]
        yl1 = yl[(xl >= xmin1) & (xl <= xmax1)]
        zl1 = zl[(xl >= xmin1) & (xl <= xmax1)]
        refL1 = refL[(xl >= xmin1) & (xl <= xmax1)]
    elif ((xmin1 == 'NoLim') & (xmax1 != 'NoLim')):
        xl1 = xl[(xl <= xmax1)]
        yl1 = yl[(xl <= xmax1)]
        zl1 = zl[(xl <= xmax1)]
        refL1 = refL[(xl <= xmax1)]
    elif ((xmin1 == 'NoLim') & (xmax1 != 'NoLim')):
        xl1 = xl[(xl >= xmin1)]
        yl1 = yl[(xl >= xmin1)]
        zl1 = zl[(xl >= xmin1)]
        refL1 = refL[(xl >= xmin1)]
    else:
        xl1 = xl
        yl1 = yl
        zl1 = zl
        refL1 = refL

    if ((ymin1 != 'NoLim') & (ymax1 != 'NoLim')):
        xl2 = xl1[(yl1 >= ymin1) & (yl1 <= ymax1)]
        yl2 = yl1[(yl1 >= ymin1) & (yl1 <= ymax1)]
        zl2 = zl1[(yl1 >= ymin1) & (yl1 <= ymax1)]
        refL2 = refL1[(yl1 >= ymin1) & (yl1 <= ymax1)]
    elif ((ymin1 == 'NoLim') & (ymax1 != 'NoLim')):
        xl2 = xl1[(yl1 <= ymax1)]
        yl2 = yl1[(yl1 <= ymax1)]
        zl2 = zl1[(yl1 <= ymax1)]
        refL1 = refL[(yl1 <= ymax1)]
    elif ((ymin1 == 'NoLim') & (ymax1 != 'NoLim')):
        xl2 = xl1[(yl1 >= ymin1)]
        yl2 = yl1[(yl1 >= ymin1)]
        zl2 = zl1[(yl1 >= ymin1)]
        refL1 = refL[(yl1 >= ymin1)]
    else:
        xl2 = xl1
        yl2 = yl1
        zl2 = zl1
        refL2 = refL1

    if ((zmin1 != 'NoLim') & (zmax1 != 'NoLim')):
        xl3 = xl2[(zl2 >= zmin1) & (zl2 <= zmax1)]
        yl3 = yl2[(zl2 >= zmin1) & (zl2 <= zmax1)]
        zl3 = zl2[(zl2 >= zmin1) & (zl2 <= zmax1)]
        refL3 = refL2[(zl2 >= zmin1) & (zl2 <= zmax1)]
    elif ((ymin1 == 'NoLim') & (ymax1 != 'NoLim')):
        xl3 = xl2[(zl2 <= zmax1)]
        yl3 = yl2[(zl2 <= zmax1)]
        zl3 = zl2[(zl2 <= zmax1)]
        refL3 = refL2[(zl2 <= zmax1)]
    elif ((ymin1 == 'NoLim') & (ymax1 != 'NoLim')):
        xl3 = xl2[(zl2 >= zmin1)]
        yl3 = yl2[(zl2 >= zmin1)]
        zl3 = zl2[(zl2 >= zmin1)]
        refL3 = refL2[(zl2 >= zmin1)]
    else:
        xl3 = xl2
        yl3 = yl2
        zl3 = zl2
        refL3 = refL2

    

        
    #Medimos los valores nulos y su proporc.
    sizeArr = M[1:,:,0].size
    numberofzeroes = np.count_nonzero(M[1:,:,0] == 0.0)
    ratiozeroes = numberofzeroes / sizeArr 
    print("Zero values: %s, Ratio zeros = %s\n" % (numberofzeroes, ratiozeroes))

    #Inic. para las longitudeas de los ejes.

    res = np.vstack((xl3,yl3,zl3,refL3))
    if (res.size == 0):
        res=np.zeros((4,1))
        dis=np.zeros((1,1))
    else:
        #res = res.reshape((4,int(res.size/4)))
        dis = np.apply_along_axis(np.linalg.norm, 0, res[0:3,:])
        dis = dis.reshape((1,int(res.size/4)))
        if (np.min(dis) <= securityDistance):
            # Common way
            print('\a')
            #NO ADMIN PRIVILEGES (linux)/ PRIVILEGES IRRELEVANT (win)        
            if (sp=='linux'):
                print('SOS')
                #os.system("beep -f 555 -l 460")
                #os.system("cw -f SOS.txt")             
                #os.system("beep -f 3500 -l 250")   #Most unpleasant sound
            elif (sp =='win'):
                ws.Beep(3500,250) #Most unpleasant sound

    #global rdp1    
    #global resultadoPos
    #global distancePoints
    #rdp1 = int(res.size/4)

    resultadoPos = res
    distancePoints = dis

    if ((pub=="y") | (pub=="s")):
        #PUblicamos en memoria mediante sqlite3
        num_rows, num_cols = resultadoPos.shape
        resultadoPosdf = pd.DataFrame(data=resultadoPos.T, index=np.arange(num_cols), columns=np.arange(num_rows))    
        num_rows, num_cols = distancePoints.shape
        distancePointsdf = pd.DataFrame(data=distancePoints.T, index=np.arange(num_cols), columns=np.arange(num_rows))
        resultadoPosdf.to_sql("resultadoPos", conn, if_exists="replace")
        distancePointsdf.to_sql("distancePoints", conn, if_exists="replace")
    else:
        pass

    return (resultadoPos, distancePoints) 

    




