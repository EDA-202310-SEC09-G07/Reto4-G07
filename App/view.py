"""
 * Copyright 2020, Departamento de sistemas y Computación, Universidad
 * de Los Andes
 *
 *
 * Desarrolado para el curso ISIS1225 - Estructuras de Datos y Algoritmos
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along withthis program.  If not, see <http://www.gnu.org/licenses/>.
 """

import config as cf
import sys
import controller
from DISClib.ADT import list as lt
from DISClib.ADT import stack as st
from DISClib.ADT import queue as qu
from DISClib.ADT import map as mp
from DISClib.DataStructures import mapentry as me
assert cf
import tabulate
import traceback

"""
La vista se encarga de la interacción con el usuario
Presenta el menu de opciones y por cada seleccion
se hace la solicitud al controlador para ejecutar la
operación solicitada
"""


def new_controller():
    """
        Se crea una instancia del controlador
    """
    #TODO OK: Llamar la función del controlador donde se crean las estructuras de datos
    control = controller.new_controller()
    return control

# ----------------------------------------------
#         Menus del sistema
# ----------------------------------------------

def print_menu():
    print("Bienvenido")
    print("1- Cargar información")
    print("2- Ejecutar Requerimiento 1")
    print("3- Ejecutar Requerimiento 2")
    print("4- Ejecutar Requerimiento 3")
    print("5- Ejecutar Requerimiento 4")
    print("6- Ejecutar Requerimiento 5")
    print("7- Ejecutar Requerimiento 6")
    print("8- Ejecutar Requerimiento 7")
    print("9- Ejecutar Requerimiento 8")
    print("0- Salir")


def menu_cant_datos(op):
    filename= None
    match op:
        case 1:
            filename="wolfs/BA-Grey-Wolf-tracks-utf8-small.csv","wolfs/BA-Grey-Wolf-individuals-utf8-small.csv"
        case 2:
            filename="wolfs/BA-Grey-Wolf-tracks-utf8-5pct.csv","wolfs/BA-Grey-Wolf-individuals-utf8-5pct.csv"
        case 3:
            filename="wolfs/BA-Grey-Wolf-tracks-utf8-10pct.csv","wolfs/BA-Grey-Wolf-individuals-utf8-10pct.csv"
        case 4:
            filename="wolfs/BA-Grey-Wolf-tracks-utf8-20pct.csv","wolfs/BA-Grey-Wolf-individuals-utf8-20pct.csv"
        case 5:
            filename="wolfs/BA-Grey-Wolf-tracks-utf8-30pct.csv","wolfs/BA-Grey-Wolf-individuals-utf8-30pct.csv"
        case 6:
            filename="wolfs/BA-Grey-Wolf-tracks-utf8-50pct.csv","wolfs/BA-Grey-Wolf-individuals-utf8-50pct.csv"
        case 7:
            filename="wolfs/BA-Grey-Wolf-tracks-utf8-80pct.csv","wolfs/BA-Grey-Wolf-individuals-utf8-80pct.csv"
        case 8:
            filename="wolfs/BA-Grey-Wolf-tracks-utf8-large.csv","wolfs/BA-Grey-Wolf-individuals-utf8-large.csv"
        case _:
            print("Opción errónea en cantidad de datos, vuelva a elegir.\n")
    return filename  
  
def print_menu_cant_datos()->int:
    print("1- Datos 0.50%")
    print("2- Datos 5%")
    print("3- Datos 10%")
    print("4- Datos 20%")
    print("5- Datos 30%")
    print("6- Datos 50%")
    print("7- Datos 80%")
    print("8- Datos 100%")
    print("0- Salir")
    op = int(input("Seleccione el tamaño de los datos que desea cargar: "))
    return op


# ----------------------------------------------
#         Print de los requerimientos
# ----------------------------------------------

def load_data(control,filename):
    """
    Carga los datos
    """
    #TODO: Realizar la carga de datos
    control, num_event, vertices_in, arcos_in, vertices, arcos, lista_lat, lista_lon, queue = controller.load_data(control, filename)
    print("\n")
    print("******Lobos y características del evento******")
    print("Número de lobos: "+ str(mp.size(control["wolfs"])))
    print("Número de lobos con datos: "+ str(mp.size(control["wolfs"])))
    print("Número de eventos: "+ str(num_event))
    print("\n")
    print("******Características de los nodos******")
    print("Puntos de encuentro: "+ str(mp.size(control["encuentros"])))
    print("Puntos de seguimiento: "+ str(vertices_in))
    print("Total: "+ str(vertices))
    print("\n")
    print("******Características de los arcos******")
    print("Arcos puntos de encuentro: "+ str(arcos-arcos_in))
    print("Arcos puntos de seguimiento: "+ str(arcos_in))
    print("Total: "+ str(arcos))
    print("\n")
    print("******Área del grafo******")
    first= lt.firstElement(lista_lat)
    last= lt.lastElement(lista_lat)

    
    print("Min y max latitudes: "+str(first)+", "+str(last))
    first= lt.firstElement(lista_lon)
    last= lt.lastElement(lista_lon)
    
    print("Min y max longitudes: "+str(last)+", "+str(first))
    
    lista= crear_lista_carga(queue)
    header = lista[0].keys()
    rows =  [x.values() for x in lista]
    print(tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= 10,maxheadercolwidths=6))
    
    return control
    
def crear_lista_carga(queue):
    lista=[]
    for i in range(qu.size(queue)):
        data= qu.dequeue(queue)
        data= data.split(";")
        dicc={
         "location-log aprox": data[0],
         "location-lat aprox": data[1], 
         "node-id": data[2],
         "individual-id": data[3],
         "adjent-nodes": data[4]   
        }
        
        lista.append(dicc)
    
    return lista
        


def print_data(control, id):
    """
        Función que imprime un dato dado su ID
    """
    #TODO: Realizar la función para imprimir un elemento
    pass

def print_req_1(control):
    """
        Función que imprime la solución del Requerimiento 1 en consola
    """
    # TODO: Imprimir el resultado del requerimiento 1
    inc= input("Ingrese el punto de encuentro de origen: ")
    fin= input("Ingrese el punto de encuentro de destino: ")
    print("Creando el árbol DFS...")
    print("Esta "+ fin + " en el arbol DFS?: ")
    valor= controller.req_1(control, inc, fin)

    if valor!= False:
        print(True)
        print("")
        lista, size, puntos_en, suma_arc= valor
        print("Total de nodos en el arbol: " + str(size))
        print("Total de puntos de encuentro en el arbol DFS: " + str(puntos_en))
        print("Total de puntos de sequimiento en el arbol DFS: " + str(size- puntos_en))
        print("Distancia total del recorrido: "+ str(round(suma_arc, 3))+ " km")
        lista= crear_lista_req1(lista)
        header = lista[0].keys()
        rows =  [x.values() for x in lista]
        print(tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= 10,maxheadercolwidths=6))
    else:
        print(valor)

def crear_lista_req1(data_structs):
    lista=[]
    for data in lt.iterator(data_structs):
        valores= data[0]
        dicc={
         "location-log aprox": valores[0],
         "location-lat aprox": valores[1], 
         "node-id": valores[2],
         "individual-id": valores[3],
         "individual-count": valores[4],   
         "edge-To": data[1],
         "edge-distance-km": data[2]
        }
        
        lista.append(dicc)
    
    return lista

def print_req_2(control):
    """
        Función que imprime la solución del Requerimiento 2 en consola
    """
    # TODO: Imprimir el resultado del requerimiento 2
    
    inc= input("Ingrese el punto de encuentro de origen: ")
    fin= input("Ingrese el punto de encuentro de destino: ")
    valor= controller.req_2(control, inc, fin)
    if valor!= False:
        print(True)
        print("")
        lista, size, puntos_en, suma_arc= valor
        print("Total de nodos en el arbol: " + str(size))
        print("Total de puntos de encuentro en el arbol BFS: " + str(puntos_en))
        print("Total de puntos de sequimiento en el arbol BFS: " + str(size- puntos_en))
        print("Distancia total del recorrido: "+ str(round(suma_arc, 3))+ " km")
        lista= crear_lista_req1(lista)
        header = lista[0].keys()
        rows =  [x.values() for x in lista]
        print(tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= 10,maxheadercolwidths=6))
    else:
        print(valor)



def print_req_3(control):
    """
        Función que imprime la solución del Requerimiento 3 en consola
    """
    # TODO: Imprimir el resultado del requerimiento 3
    pass


def print_req_4(control):
    """
        Función que imprime la solución del Requerimiento 4 en consola
    """
    # TODO: Imprimir el resultado del requerimiento 4
    pass


def print_req_5(control):
    """
        Función que imprime la solución del Requerimiento 5 en consola
    """
    # TODO: Imprimir el resultado del requerimiento 5
    puntos= input("Ingrese el mínimo de puntos de encuentro a visitar: ")
    kil= input("Ingrese los kilometros a recorrer: ")
    
    inc= input("Ingrese el punto de origen: ")
    valor= controller.req_5(control, puntos, kil, inc)
    if valor!= False:
        rutas, min_pun, distancia, queue= valor
        print("Hay "+ str(rutas)+ " posibles rutas desde el punto: "+ inc)
        print("El mínimo número de puntos de encuentro a visitar es: "+ str(min_pun))
        print("La máxima distancia recorrer es de: "+ str(distancia))
        print("*****Destalles del camino más largo*****")
        lista= crear_lista_req_5(queue)
        header = lista[0].keys()
        rows =  [x.values() for x in lista]
        print(tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= 10,maxheadercolwidths=6))
    else:
        print(valor)

def crear_lista_req_5(queue):
    lista=[]
    dicc={}
    while not qu.isEmpty(queue):
        data= qu.dequeue(queue)
        if qu.size(queue)>3:
            dicc["Points Count"]= data
        elif qu.size(queue)>2:
            dicc["Path distance [km]"]= data
        elif qu.size(queue)>1:
            for point in lt.iterator(data):
                list_points= list_points+","+ point
            list_points= list_points.strip(",")
            dicc["Point List"]= list_points
        else:
            for animals in lt.iterator(data):
                list_animal= list_animal+","+ animals
            list_animal= list_animal.strip(",")
            dicc["Animal Count"]= list_animal
    lista.append(dicc)
    return lista

def print_req_6(control):
    """
        Función que imprime la solución del Requerimiento 6 en consola
    """
    # TODO: Imprimir el resultado del requerimiento 6
    pass


def print_req_7(control):
    """
        Función que imprime la solución del Requerimiento 7 en consola
    """
    # TODO: Imprimir el resultado del requerimiento 7
    pass


def print_req_8(control):
    """
        Función que imprime la solución del Requerimiento 8 en consola
    """
    # TODO: Imprimir el resultado del requerimiento 8
    pass


# Se crea el controlador asociado a la vista
control = new_controller()

# main del reto
if __name__ == "__main__":
    """
    Menu principal
    """
    working = True
    #ciclo del menu
    while working:
        print_menu()
        inputs = input('Seleccione una opción para continuar\n')
        try:
            if int(inputs) == 1:
                print("Cargando información de los archivos ....\n")
                op = print_menu_cant_datos()
                filename = menu_cant_datos(op)
                data = load_data(control, filename)
            elif int(inputs) == 2:
                print_req_1(control)

            elif int(inputs) == 3:
                print_req_2(control)

            elif int(inputs) == 4:
                print_req_3(control)

            elif int(inputs) == 5:
                print_req_4(control)

            elif int(inputs) == 6:
                print_req_5(control)

            elif int(inputs) == 7:
                print_req_6(control)

            elif int(inputs) == 8:
                print_req_7(control)

            elif int(inputs) == 9:
                print_req_8(control)

            elif int(inputs) == 0:
                working = False
                print("\nGracias por utilizar el programa")
                
            else:
                print("Opción errónea, vuelva a elegir.\n")
        except Exception as exp:
            print("ERR:", exp)
            traceback.print_exc()
    sys.exit(0)
