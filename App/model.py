"""
 * Copyright 2020, Departamento de sistemas y Computación,
 * Universidad de Los Andes
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
 *
 * Contribuciones:
 *
 * Dario Correal - Version inicial
 """


import config as cf
from DISClib.ADT import list as lt
from DISClib.ADT import stack as st
from DISClib.ADT import queue as qu
from DISClib.ADT import map as mp
from DISClib.ADT import minpq as mpq
from DISClib.ADT import indexminpq as impq
from DISClib.ADT import orderedmap as om
from DISClib.DataStructures import mapentry as me
from DISClib.ADT import graph as gr
from DISClib.Algorithms.Graphs import scc
from DISClib.Algorithms.Graphs import dijsktra as djk
from DISClib.Algorithms.Graphs import bellmanford as bf
from DISClib.Algorithms.Graphs import bfs
from DISClib.Algorithms.Graphs import dfs
from DISClib.Algorithms.Graphs import prim
from DISClib.Algorithms.Sorting import shellsort as sa
from DISClib.Algorithms.Sorting import insertionsort as ins
from DISClib.Algorithms.Sorting import selectionsort as se
from DISClib.Algorithms.Sorting import mergesort as merg
from DISClib.Algorithms.Sorting import quicksort as quk
from math import radians, cos, sin, asin, sqrt
assert cf

"""
Se define la estructura de un catálogo de videos. El catálogo tendrá
dos listas, una para los videos, otra para las categorias de los mismos.
"""

# Construccion de modelos

def new_list():
    lista= lt.newList(datastructure="ARRAY_LIST")
    return lista



def new_data_structs(control):
    """
    Inicializa las estructuras de datos del modelo. Las crea de
    manera vacía para posteriormente almacenar la información.
    """
    #TODO: Inicializar las estructuras de datos
    control["wolfs"]= mp.newMap(20,
                                        maptype='PROBING',
                                        loadfactor=0.5,
                                        cmpfunction=compare_map)
    
    control["positions"]= mp.newMap(3,
                                    maptype='PROBING',
                                    loadfactor=0.5,
                                    cmpfunction=compare_map)
    
    control["encuentros"]= mp.newMap(20,
                                    maptype='PROBING',
                                    loadfactor=0.5,
                                    cmpfunction=compare_map)
    
    control["moves"]= gr.newGraph(datastructure= "ADJ_LIST",
                                      directed= True,
                                      size=300000,
                                      cmpfunction=compareStopIds)
    return control
    
#funcion temporal xd
def compare_map(data_1, data_2):
    data_2=me.getKey(data_2)
    if data_1 > data_2:
        return 1
    elif data_1 < data_2:
        return -1
    else:
        return 0

def compareStopIds(stop, keyvaluestop):
    """
    Compara dos estaciones
    """
    stopcode = keyvaluestop['key']
    if (stop == stopcode):
        return 0
    elif (stop > stopcode):
        return 1
    else:
        return -1

# Funciones para agregar informacion al modelo

def load_moves(control,lista_eventos):
    """
    Función para agregar nuevos elementos a la lista
    """
    #TODO: Crear la función para agregar elementos a una lista
    #agregamos los nodos y los vertices
    grafo= control["moves"]
    mapa= control["positions"]
    first= lt.firstElement(lista_eventos)
    individual_id=first["individual-local-identifier"]+"_"+first["tag-local-identifier"]
    pos=1
    lista_eventos2=lista_eventos
    for evento in lt.iterator(lista_eventos2):
        punto= crear_identificador(evento)
        mp.put(mapa, punto, evento)
        if pos== 1:
            gr.insertVertex(grafo, punto)
        elif individual_id== (evento["individual-local-identifier"]+"_"+evento["tag-local-identifier"]):
            gr.insertVertex(grafo, punto)
            evento_anterior= lt.getElement(lista_eventos, pos)
            lat1= round(float(evento["location-lat"]), 3)
            lon1=round(float(evento["location-long"]), 3)
            lat2= round(float(evento_anterior["location-lat"]), 3)
            lon2=round(float(evento_anterior["location-long"]), 3)
            peso= haversine(lon1, lat1, lon2, lat2)
            punto_2= crear_identificador(evento_anterior)
            gr.addEdge(grafo, punto_2, punto, peso)
            
        else: 
            individual_id=evento["individual-local-identifier"]+"_"+evento["tag-local-identifier"]
            gr.insertVertex(grafo, punto)
        
        pos+=1
            
    
    control["moves"]= grafo  
    control["positions"] = mapa      

    return control

def agregar_encuentros(control, lista_eventos):
    grafo= control["moves"]
    mapa= control["encuentros"]
    pos=1
    first= lt.firstElement(lista_eventos)
    lista_eventos2=lista_eventos
    wolf=first["individual-local-identifier"]+"_"+first["tag-local-identifier"]
    for evento in lt.iterator(lista_eventos2):
        lat1= round(float(evento["location-lat"]), 3)
        lon1=round(float(evento["location-long"]), 3)
        if pos != lt.size(lista_eventos):
            evento_siquiente= lt.getElement(lista_eventos, pos+1)
            lat2= round(float(evento_siquiente["location-lat"]), 3)
            lon2=round(float(evento_siquiente["location-long"]), 3)
            if lon1 == lon2 and lat1== lat2:
                if wolf != evento["individual-local-identifier"]+"_"+evento["tag-local-identifier"]:
                    wolf=evento["individual-local-identifier"]+"_"+evento["tag-local-identifier"]
                    tag_lat= str(lat1)
                    tag_lat= tag_lat.replace("-", "m")
                    tag_lat= tag_lat.replace(".", "p")
                    tag_lon= str(lon1)
                    tag_lon= tag_lon.replace("-", "m")
                    tag_lon= tag_lon.replace(".", "p")
                    tag_ver= tag_lon+"_"+tag_lat
                    punto1= crear_identificador(evento)
                    punto2= crear_identificador(evento_siquiente)
                    if gr.containsVertex(grafo, tag_ver):
                        gr.addEdge(grafo, tag_ver, punto2, 0.000)
                        gr.addEdge(grafo, punto2, tag_ver, 0.000)
                    else:
                        valor= lon1, lat1
                        mp.put(mapa, tag_ver, valor)
                        gr.insertVertex(grafo, tag_ver)
                        gr.addEdge(grafo, tag_ver, punto1, 0.000)
                        gr.addEdge(grafo, punto1, tag_ver, 0.000)
                        gr.addEdge(grafo, tag_ver, punto2, 0.000)
                        gr.addEdge(grafo, punto2, tag_ver, 0.000)
                    
        pos+=1
    
    control["moves"]= grafo
    control["encuentros"]= mapa
    
    return control     
        
    


# Funciones para creacion de datos
def haversine(lon1, lat1, lon2, lat2):
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371
    return c * r

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
    
def add_wolfs(control, wolf):
    individual_id=wolf["animal-id"]+"_"+wolf["tag-id"]
    mp.put(control["wolfs"] ,individual_id,wolf)
    return control

def add_list_evento(data,lista_eventos):
    lt.addLast(lista_eventos,data)
    return lista_eventos
    
# Funciones de consulta

def get_data(data_structs, id):
    """
    Retorna un dato a partir de su ID
    """
    #TODO: Crear la función para obtener un dato de una lista
    pass


def data_size(data_structs):
    """
    Retorna el tamaño de la lista de datos
    """
    #TODO: Crear la función para obtener el tamaño de una lista
    return lt.size(data_structs)


def req_1(data_structs):
    """
    Función que soluciona el requerimiento 1
    """
    # TODO: Realizar el requerimiento 1
    pass


def req_2(data_structs):
    """
    Función que soluciona el requerimiento 2
    """
    # TODO: Realizar el requerimiento 2
    pass


def req_3(data_structs):
    """
    Función que soluciona el requerimiento 3
    """
    # TODO: Realizar el requerimiento 3
    pass


def req_4(data_structs):
    """
    Función que soluciona el requerimiento 4
    """
    # TODO: Realizar el requerimiento 4
    pass


def req_5(data_structs):
    """
    Función que soluciona el requerimiento 5
    """
    # TODO: Realizar el requerimiento 5
    pass


def req_6(data_structs):
    """
    Función que soluciona el requerimiento 6
    """
    # TODO: Realizar el requerimiento 6
    pass


def req_7(data_structs):
    """
    Función que soluciona el requerimiento 7
    """
    # TODO: Realizar el requerimiento 7
    pass


def req_8(data_structs):
    """
    Función que soluciona el requerimiento 8
    """
    # TODO: Realizar el requerimiento 8
    pass


# Funciones utilizadas para comparar elementos dentro de una lista

def compare(data_1, data_2):
    """
    Función encargada de comparar dos datos
    """
    #TODO: Crear función comparadora de la lista
    pass

# Funciones de ordenamiento


def sort_criteria(data_1, data_2):
    """sortCriteria criterio de ordenamiento para las funciones de ordenamiento

    Args:
        data1 (_type_): _description_
        data2 (_type_): _description_

    Returns:
        _type_: _description_
    """
    #TODO: Crear función comparadora para ordenar
    pass

def sort_event(data1,data2):
    if data1["individual-local-identifier"] < data2["individual-local-identifier"]:
        return True
    elif data1["individual-local-identifier"] == data2["individual-local-identifier"]:
        if data1["tag-local-identifier"] < data2["tag-local-identifier"]:
            return True
        elif data1["tag-local-identifier"] == data2["tag-local-identifier"]:
            return data1["timestamp"] < data2["timestamp"]
        else:
            False
    else:
        return False
    
def sort_lon_lat(data1,data2):
    
    if round(float(data1["location-long"]), 3)< round(float(data2["location-long"]), 3):
        return True
    elif round(float(data1["location-long"]), 3)==round(float(data2["location-long"]), 3):
        if round(float(data1["location-lat"]), 3) < round(float(data2["location-lat"]), 3):
            return True
        elif round(float(data1["location-lat"]), 3) == round(float(data2["location-lat"]), 3):
            return data1["individual-local-identifier"] < data2["individual-local-identifier"]
        else: 
            return False
    else: 
        return False

def sort(data_structs, num):
    """
    Función encargada de ordenar la lista con los datos
    """
    #TODO: Crear función de ordenamiento
    if num ==1:
        data_structs= merg.sort(data_structs, sort_event)
    elif num ==2:
        data_structs= merg.sort(data_structs, sort_lon_lat)
    return data_structs
