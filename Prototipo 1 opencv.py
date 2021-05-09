# -*- coding: utf-8 -*-
"""
@author: Nicolas Barragan
"""

import cv2 as cv
import numpy as np
import imutils as im
import pyautogui

#mapear valores de posicion del centroide de la mano con el puntero en el screen
def mapear(valor, start1, stop1, start2, stop2):
   return (valor - start1) / (stop1 - start1) * (stop2 - start2) + start2

#captura
cap = cv.VideoCapture(0) #acceso a la camara integrada del computador

#captura fondo (para luego separarla del plano de la mano)
fondo = None

#transformaciones morfologicas
kernel= cv.getStructuringElement(cv.MORPH_ELLIPSE, (10,10))

#Guardara en que estado se encuentra actualmente el programa
# 0: Defecto, 1: Start, 2: Update, 3: Error
estado = 0 

#ciclo infinito "update"
while True:
    
    #verificar si la captura esta funcionando
    disponible, fotograma = cap.read()
    
    #Maquina de estados
    if(disponible == True and estado==0):
        estado = 1 #Empieza el programa
    elif(disponible == False and estado==0):
        estado = 3 #Error(camara no disponible o camara dañada)
        
    #Estados
    if(estado == 1): #Start
        print("Start")
        
        estado = 2

    elif(estado == 2): #Update
        #print("Update")
                
        #Redimensionar la imagen para que tenga un ancho de 640
        fotograma = im.resize(fotograma,width=640)
        fotograma = cv.flip(fotograma,1)
        Auxfotograma = fotograma.copy()        
        
        #Obtencion imagen binaria
        if(fondo is not None):
            
            #mostrar el fondo
            #cv.imshow("Fondo", fondo)
            
            #Region de interes(ROI) del objeto (mano)
            #print("ROI objeto")
            ROI = fotograma[50:300,380:600] #[y1:y2,x1:x2]
            cv.rectangle(fotograma,(380-2,50-2),(600+2,300+2), (0,0,255),1)
            ROIgris = cv.cvtColor(ROI, cv.COLOR_BGR2GRAY)
            
            #Region de interes del fondo
            #print("ROI fondo")
            ROIfondo = fondo[50:300,380:600] #[y1:y2,x1:x2]
            
            #Determinar imagen binaria (fondo, objeto)
            diferencia = cv.absdiff(ROIgris, ROIfondo)
            _, umbral = cv.threshold(diferencia, 30, 255, cv.THRESH_BINARY)
            umbral = cv.medianBlur(umbral,5)
            salida = cv.morphologyEx(umbral,cv.MORPH_OPEN,kernel,3)
            
            #mostrar las regiones de interes
            #cv.imshow("ROI objeto", ROI)
            #cv.imshow("ROI gris", ROIgris)
            #cv.imshow("ROI fondo", ROIfondo)
            
            #mostrar la imagen binaria
            #cv.imshow("diferencia", diferencia)
            #cv.imshow("umbral", umbral)
            #cv.imshow("salida", salida)
         
            #Obtencion contornos
            #contornos, _ = cv.findContours(umbral, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            contornos, _ = cv.findContours(salida, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            contornos = sorted(contornos, key = cv.contourArea, reverse = True)[:1] #obtener el contorno mas grande
            #cv.drawContours(ROI, contornos, 0, (255,0,0), 1)
        
            #Calcular centroide
            for c in contornos:
                M = cv.moments(c)
                if M["m00"] == 0: M["m00"]=1
                x = int(M["m10"]/M["m00"])
                y = int(M["m01"]/M["m00"])
                #dibujar un punto en el centroide
                cv.circle(ROI,(x,y),5,(0,255,0),-1)
                #mostrar las coordenadas del centroide
                fuente = cv.FONT_HERSHEY_SIMPLEX
                cv.putText(fotograma,'{},{}'.format(x,y),(x+10,y), fuente, 0.75, (0,255,0), 1, cv.LINE_AA)
                if((x and y) is not None):
                    #Obtener el tamaño del screen
                    w,h = pyautogui.size()
                    #mapear posicion del mouse con la posicion del centroide (como en processing)
                    mx = mapear(x,60,140,50,w-100)
                    my = mapear(y,110,175,50,h-200)
                    #Mover el mouse
                    pyautogui.FAILSAFE=False #evitar errores cuando se esta en los bordes del screen
                    pyautogui.moveTo(mx,my) #el cursor del mouse se mueve a la direccion indicada
                #Dibujar Contorno encontrado a traves de convex Hull
                hull = cv.convexHull(c)
                cv.drawContours(ROI, [hull], 0, (0,255,0), 2)
                #calcular defectos de convexidad, para saber si se abrio o no la mano
                hull2= cv.convexHull(c, returnPoints=False)
                defectos = cv.convexityDefects(c,hull2)
                #ver si hay defectos de convexidad
                if defectos is not None:
                    inicio = []
                    for i in range(defectos.shape[0]):
                        #información dada por el defecto de convexidad
                        s,e,f,d = defectos[i,0] #s(puntoInicial),e(puntoFinal),f(puntoMasAlejado),d(distanciaentre el punto inicial y el mas alejado)
                        puntoInicial = c[s][0] 
                        puntoFinal = c[e][0]
                        puntoAlejado = c[f][0]
                        
                        #distancia entre los puntos inicial  y final
                        if(np.linalg.norm(puntoInicial-puntoFinal)>20  and d > 12000 and cv.contourArea(hull)>20000):
                            #guardar los puntos
                            inicio.append(puntoInicial)
                            #pintar los puntos dados por los defectos de convexidad
                            cv.circle(ROI, tuple(puntoInicial), 5, (255,0,0),2)
                            cv.circle(ROI, tuple(puntoFinal), 5, (255,255,0),2)
                            cv.circle(ROI, tuple(puntoAlejado), 5, (0,255,255),2)
                
                    # Si no se han almacenado puntos de inicio (o fin), puede tratarse de
                    # 0 dedos levantados (puño cerrado)
                    if len(inicio)==0:
                        #oprimir el mouse
                        print("Click Mouse")
                        pyautogui.mouseDown()
                    else:
                        #liberar el mouse
                        print("Release Mouse")
                        pyautogui.mouseUp()
                    
        #Mostrar la captura
        cv.imshow("Camara", fotograma)
        
            
        
        #Comandos de teclado
        if(cv.waitKey(1) & 0xFF == ord('q')):
            #Liberar camara y destruir ventanas
            print("Finalizando")
            break
        elif(cv.waitKey(10) & 0xFF == ord('c')):
            #Calibrando(para determinar la imagen binaria)
            print("Calibrando")
            fondo = cv.cvtColor(Auxfotograma, cv.COLOR_BGR2GRAY)
        
    elif(estado == 3): #Error
        print("Error, camara no disponible")
        break
        
#libera camara y destruye ventanas
cap.release()
cv.destroyAllWindows()      
        
    