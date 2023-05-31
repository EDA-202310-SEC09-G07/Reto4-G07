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
        if len(data)==3:
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
    print("calculando los territorios de los lobos con componentes fuertemente conectados")
    valor= controller.req_3(control)
    print(valor)
    print(valor["value"])
    print("llegue al final xd")
    #lista_eventos= crear_lista_req4(valor)
    #header = lista_eventos[0].keys()
    #rows =  [x.values() for x in lista_eventos]
    #print(tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= 10,maxheadercolwidths=6))


def print_req_4(control):
    """
        Función que imprime la solución del Requerimiento 4 en consola
    """
    # TODO: Imprimir el resultado del requerimiento 4
    ori_lon = input("Ingrese la longitud del punto de origen: ")
    ori_lat = input("Ingrese la latitud del punto de origen: ")
    des_lon = input("Ingrese la longitud del punto de destino: ")
    des_lat = input("Ingrese la latitud del punto de destino: ")
    dist_ori, dist_des, costo, num_nodos, num_lobos, num_arcos, lista_final = controller.req_4(control, ori_lon, ori_lat, des_lon, des_lat)
    print()
    print("Distancia del punto de encuentro de origen al punto de origen: " + str(dist_ori) + " km")
    print()
    print("Distancia del punto de encuentro de destino al punto de destino: " + str(dist_des) + " km")
    print()
    print("Distancia recorrida entre los puntos de encuentro de origen y destino: " + str(costo) + " km")
    print()
    print("Total de puntos de encuentro que pertenecen al camino identificado: " + str(num_nodos))
    print()
    print("Total de individuos/lobos distintos que utilizan el corredor identificado: " + str(num_lobos))
    print()
    print("Total de segmentos que conforman la ruta identificada: " + str(num_arcos))
    print()
    lista_eventos= crear_lista_req4(lista_final)
    header = lista_eventos[0].keys()
    rows =  [x.values() for x in lista_eventos]
    print(tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= 10,maxheadercolwidths=6))

def crear_lista_req4(data_structs):
    lista=[]
    
    for data in lt.iterator(data_structs):
        dicc={
            "node-id": data[2],
            "location-long-aprox": round(data[0], 3),
            "location-lat-aprox": round(data[1], 3),
            "individual-id": data[3],
            "individual-count": data[4],
            "next-node": data[5]
        }
        lista.append(dicc)
        
    return lista
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
        rutas, min_pun, distancia, respuesta= valor
        print("Hay "+ str(rutas)+ " posibles rutas desde el punto: "+ inc)
        print("El mínimo número de puntos de encuentro a visitar es: "+ str(min_pun))
        print("La máxima distancia recorrer es de: "+ str(distancia))
        print("*****Destalles del camino más largo*****")
        lista= crear_lista_req_5(respuesta)
        header = lista[0].keys()
        rows =  [x.values() for x in lista]
        print(tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= 10,maxheadercolwidths=6))
    else:
        print(valor)

def crear_lista_req_5(valor):
    lista=[]
    
    list_animal=""
    for animals in lt.iterator(valor[3]):
        list_animal= list_animal+","+ str(animals)
    list_animal= list_animal.strip(",")
    
    list_points=""
    for point in lt.iterator(valor[2]):
        list_points= list_points+","+ point
    list_points= list_points.strip(",")
    
    dicc={ "Point Count": valor[0],
          "Path distance [km]": valor[1],
          "Point List": list_points,
          "Animal Count": list_animal
        
    }
    lista.append(dicc)
    return lista

def print_req_6(control):
    """
        Función que imprime la solución del Requerimiento 6 en consola
    """
    # TODO: Imprimir el resultado del requerimiento 6
    inc= input("Ingrese la fecha inicial: ")
    fin= input("Ingrese la fecha final: ")
    gen= input("Ingrese el género: ")
    mayor, iden_may, wolf_may, lista_may, nodos_may, menor, iden_men, wolf_men, lista_men, nodos_men, arcos, vertices= controller.req_6(control, inc, fin, gen)
    print("Creando el nuevo grafo...")
    print("Los vertices del nuevo grafo son: "+ str(vertices))
    print("Los arcos del nuevo grafo son: "+ str(arcos))
    print("*****Detalles para el lobo viajero más largo y cercano****")
    print("El individuo con la mayor distancia de viaje es:")
    print("         -Individual id: "+ str(iden_may))
    
    lista_wolf= []
    dicc_wolf= {"individual-id": iden_may,
                "animal-taxon" : wolf_may["animal-taxon"],
                "animal-life-stage": wolf_may["animal-life-stage"],
                "animal sex": wolf_may["animal-sex"],
                "study-site": wolf_may["study-site"],
                "travel-dist": mayor,
                "deployment-comments": wolf_may["deployment-comments"]        
    }
    lista_wolf.append(dicc_wolf)
    header = lista_wolf[0].keys()
    rows =  [x.values() for x in lista_wolf]
    print(tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= 10,maxheadercolwidths=6))
    print("")
    print("El camino más largo para el lobo "+ str(iden_may)+ " tiene:")
    print("         -Nodos:"+ str(nodos_may))
    print("         -Arcos:"+ str(nodos_may-1))
    print("         -Distancia: "+ str(mayor))
    print("")
    print("***************************************")
    print("Los primeros 3 y ultimos 3 nodos son: ")
    
    lista_eventos= crear_lista_req6(lista_may)
    header = lista_eventos[0].keys()
    rows =  [x.values() for x in lista_eventos]
    print(tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= 10,maxheadercolwidths=6))
    
    
    
    
    print("El individuo con la menor distancia de viaje es:")
    print("         -Individual id: "+ str(iden_men))
    lista_wolf= []
    dicc_wolf= {"individual-id": iden_men,
                "animal-taxon" : wolf_men["animal-taxon"],
                "animal-life-stage": wolf_men["animal-life-stage"],
                "animal sex": wolf_men["animal-sex"],
                "study-site": wolf_men["study-site"],
                "travel-dist": menor,
                "deployment-comments": wolf_men["deployment-comments"]        
    }
    lista_wolf.append(dicc_wolf)
    header = lista_wolf[0].keys()
    rows =  [x.values() for x in lista_wolf]
    print(tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= 10,maxheadercolwidths=6))
    print("")
    print("El camino más largo para el lobo "+ str(iden_men)+ " tiene:")
    print("         -Nodos:"+ str(nodos_men))
    print("         -Arcos:"+ str(nodos_men-1))
    print("         -Distancia: "+ str(menor))
    print("")
    print("***************************************")
    print("Los primeros 3 y ultimos 3 nodos son: ")
    lista_eventos= crear_lista_req6(lista_men)
    header = lista_eventos[0].keys()
    rows =  [x.values() for x in lista_eventos]
    print(tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= 10,maxheadercolwidths=6))


def crear_lista_req6(data_structs):
    lista=[]
    
    for data in lt.iterator(data_structs):
        dicc={
            "node-id": crear_identificador(data),
            "location-long-aprox": round(float(data["location-long"]), 3),
            "location-lat-aprox": round(float(data["location-lat"]), 3),
            "individual-id": data["individual-local-identifier"]+"_"+data["tag-local-identifier"],
            "individual-count": 1
        }
        lista.append(dicc)
        
    return lista
        
        
def crear_identificador(data):
    """ Crea el identificador id para agregar al grafo

    Args:
        data (dict): recibe el dato al cual se le va a crear el identificador 
    """
    lat= round(float(data["location-lat"]), 3)
    lat= str(lat)
    lat= lat.replace("-", "m")
    lat= lat.replace(".", "p")
    
    lon=round(float(data["location-long"]), 3)
    lon= str(lon)
    lon= lon.replace("-", "m")
    lon= lon.replace(".", "p")
    
    individual_id=data["individual-local-identifier"]+"_"+data["tag-local-identifier"]
    
    punto= lon+"_"+lat+"_"+individual_id
    
    return punto
def print_req_7(control):
    """
        Función que imprime la solución del Requerimiento 7 en consola
    """
    # TODO: Imprimir el resultado del requerimiento 7
    inc= input("Ingrese la fecha inicial: ")
    fin= input("Ingrese la fecha final: ")
    tep_min= input("Ingrese la temperatura mínima: ")
    tep_max= input("Ingrese la temperatura máxima: ")
    vertices, arcos, sccs, lista_final= controller.req_7(control, inc, fin, tep_min, tep_max)
    size= [11,11,11,11,11,11,None,None]
    lista1= crear_lista_req_7(lista_final)
    header = lista1[0].keys()
    rows =  [x.values() for x in lista1]
    print(tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= size,maxheadercolwidths=size))
    
    lista2=crear_lista_req_7_2(lista_final)
    header = lista2[0].keys()
    rows =  [x.values() for x in lista2]
    print(tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= size,maxheadercolwidths=size))
    
def crear_lista_req_7(lista_final):
    lista=[]
    for mapa in lt.iterator(lista_final):
        dicc={
            "SCCID": me.getValue(mp.get(mapa, "sccid")),
            "Dips Node IDs": me.getValue(mp.get(mapa, "nodesid")),
            "SCC size": me.getValue(mp.get(mapa, "sccsize")), 
            "min-lat": me.getValue(mp.get(mapa, "min-lat")),
            "max-lat": me.getValue(mp.get(mapa, "max-lat")),
            "min-lon": me.getValue(mp.get(mapa, "min-lon")),
            "max-lon": me.getValue(mp.get(mapa, "max-lon")),
            "Wolf Count": me.getValue(mp.get(mapa, "wolfcount")),
            "Wolf Details": crear_tabla_lobos_req_7(me.getValue(mp.get(mapa, "wolfdetails")))
            
        }
        lista.append(dicc)
        
    return lista

def crear_tabla_lobos_req_7(wolfs):
    lista=[]
    for wolf in lt.iterator(wolfs):
        dicc={
            "individual-id": wolf["animal-id"]+"_"+wolf["tag-id"],
            "animal sex": wolf["animal-sex"],
            "animal-life-stage": wolf["animal-life-stage"],
            "study-site": wolf["study-site"],
            "deployment-comments": wolf["deployment-comments"]
        }
        
        lista.append(dicc)
    header = lista[0].keys()
    rows =  [x.values() for x in lista]
    tabla= tabulate.tabulate(rows,header,tablefmt="grid",maxcolwidths= 10,maxheadercolwidths=6)
    return tabla


def crear_lista_req_7_2(lista_final):
    lista=[]
    for mapa in lt.iterator(lista_final):
        dicc={
            "SCCID": me.getValue(mp.get(mapa, "sccid")),
            "SCC size": me.getValue(mp.get(mapa, "sccsize")), 
            "min-lat": me.getValue(mp.get(mapa, "min-lat")),
            "max-lat": me.getValue(mp.get(mapa, "max-lat")),
            "min-lon": me.getValue(mp.get(mapa, "min-lon")),
            "max-lon": me.getValue(mp.get(mapa, "max-lon")),
            "LP Node Count": me.getValue(mp.get(mapa, "nodes")),
            "LP Edges Count": me.getValue(mp.get(mapa, "edges")),
            "LP distance [km]": me.getValue(mp.get(mapa, "distance")),
            "Dips Disp details": me.getValue(mp.get(mapa, "nodesid"))
            
        }
        lista.append(dicc)
        
    return lista
def print_req_8(control):
    """
        Función que imprime la solución del Requerimiento 8 en consola
    """
    inputs = input('Seleccione que requerimiento desea ejecutar\n')
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
