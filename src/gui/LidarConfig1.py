import socket
import struct
import numpy as np
import getlidarMain
        
class LidarConfig1():
   #Constructor de la clase. Necesita instanciarse con un socket (MSOP), un socket (DIFOP), la matriz de ang. de altura, el time offset y la velocidad del LiDAR
   # La clase tiene como fin el poder pasar todos estos param. al cuerpo de la func. de lectura. 
   # La conceptualizac. como clase nos permite instanciar varios objetos de esta clase para el caso de usar varios LiDAR simult. llegado el momento.
   def __init__(self, UDP_IP, UDP_PORT, UDP_PORT_DIFOP,graphadj,secdis):
        # Protocolo UDP socket MSOP (datos)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Enlazando (binding) el socket. El socket se pasa a la func. para hacer visible la con. desde la misma.
        sock.bind((UDP_IP, UDP_PORT))
        self.msopsocket = sock
        # Protocolo UDP socket DIFOP (config.)
        sock_difop = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Enlazando (binding) el socket. El socket se pasa a la func. para hacer visible la con. desde la misma.
        sock_difop.bind((UDP_IP, UDP_PORT_DIFOP))
        #DIFOP
        self.difopsocket = sock_difop
        data_difop, addr = sock_difop.recvfrom(1290)
        #Leemos un paquete DIFOP. Hemos de leer los ang. calibrados de altura.
        #Siguiendo al manual v 4.3.3. (pags 25 y 43) el offset es 1165, son 48 bytes, y cada ang. viene codificado por tres bytes.
        a = np.arange(1165, 1165 + 48, 3)
        indexC = 0
        corr_h = np.zeros(16)
        for i in a:
            if (indexC > 7):
                corr_h[indexC] = getlidarMain.my_hex_unpack1(data_difop[i:i+3]) * 1.e-4
            else:
                corr_h[indexC] = -1. * getlidarMain.my_hex_unpack1(data_difop[i:i+3]) * 1.e-4
                indexC = indexC + 1        
        #Creamos la matriz bas. de ang. de altura.
        altMat = np.repeat(np.array(corr_h),12).reshape(16,12)
        #Hay que extraer del paquete DIFOP anterior la velocidad de rot. Determina el tam. de las matrices.
        rotationSpeed = struct.unpack(">H", bytearray(data_difop[8:8+2]))[0]
        motorPhaseLock = struct.unpack(">H", bytearray(data_difop[38:38+2]))[0]
        lidarSpeed = rotationSpeed
        #Resol. esp. de la distancia. Valor por el que hay que multiplicar las distancias obtenidas en los paquetes MSOP (datos)
        resoL1 = 0.5e-2
        #Func. para calcular el time offset para cada bloque y disparo respecto al valor inicial indicado en el paquete MSOP.
        #Creamos la matriz de offset de tiempos mediante la func. definida anteriormente. Hay que pasarla a la func. mediante la Clase LidarConfig.
        t_offset = np.zeros((32,12))
        for i in range(12):
            for k in range(32):
                t_offset[k,i] = getlidarMain.time_offset1(k, i)
                #t_offset[k,i] = time_offset(firing, block)
        #Creamos un objeto de la clase LidarConfig para poder pasar los sockets, la mat de ang de altura, el time offset y la velocidad de rot. del LiDAR a la func. de lectura.            
        
        self.altMat1 = altMat
        self.t_offset = t_offset
        self.lidarSpeed = lidarSpeed
        self.graphadj = graphadj
        self.secdis = secdis
   
   # Get Methods    
   def getMsopSocket(self):
       return(self.msopsocket)
       
   def getDifopSocket(self):
       return(self.difopsocket)
       
   def getAltMat1(self):
       return(self.altMat1)
       
   def get_t_offset(self):
       return(self.t_offset)
       
   def getLidarSpeed(self):
       return(self.lidarSpeed)
   
   def getGraphAdj(self):
       return(self.graphadj)
       
   def getSecDis(self):
       return(self.secdis)



   # Set Methods. Se han creado para contemplar la posibilidad - altamente improbable - de cambios en tiempo de ejecuc.    
   def setMsopSocket(newMsopSocket):
       self.msopsocket = newMsopSocket
       
   def setDifopSocket(newDifopSocket):
       self.difopsocket = newDifopSocket
       
   def setAltMat1(newAltMat1):
       self.altMat1 = newAltMat1
       
   def set_t_offset(newT_Offset):
       self.t_offset = newT_Offset
       
   def setLidarSpeed(newLidarSpeed):
       self.lidarSpeed = newLidarSpeed

   def setGraphAdj(newGraphAdj):
       self.graphadj = newGraphAdj

   def setSecDis(newSecDis):
       self.secdis = newSecDis
