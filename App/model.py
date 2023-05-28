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
    
    control["encuentros"]= mp.newMap(66,
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

def load_moves(control, lista_eventos):
    """
    Función para agregar nuevos elementos a la lista
    """
    mapa = control["positions"]
    grafo = control["moves"]

    anterior = None

    for evento in lt.iterator(lista_eventos):
        punto = crear_identificador(evento)
        individual_id = evento["individual-local-identifier"] + "_" + evento["tag-local-identifier"]
        if not gr.containsVertex(grafo, punto):
            gr.insertVertex(grafo, punto)
            mp.put(mapa, punto, evento)
        
        if anterior is not None and individual_id == anterior["individual-local-identifier"] + "_" + anterior["tag-local-identifier"]:
            punto_ant = crear_identificador(anterior)
            if gr.getEdge(grafo, punto_ant, punto)== None:
                lon1 = round(float(anterior["location-long"]), 3)
                lat1 = round(float(anterior["location-lat"]), 3)
                lon2 = round(float(evento["location-long"]), 3)
                lat2 = round(float(evento["location-lat"]), 3)

                peso = haversine(lon1, lat1, lon2, lat2)
                gr.addEdge(grafo, punto_ant, punto, peso)
        
        anterior = evento

    control["positions"] = mapa
    control["moves"] = grafo

    return control, gr.numVertices(grafo), gr.numEdges(grafo)


    
            
        

def agregar_encuentros(control):
    grafo = control["moves"]
    mapa = control["encuentros"]
    mapa_positions = control["positions"]
    
    lista = mp.keySet(mapa_positions)
    lista = sort(lista, 2)
    anterior = None

    for punto in lt.iterator(lista):
        if anterior is not None:
            iden1, lon1, lat1 = obtener_identificador_lon_lat(anterior)
            iden2, lon2, lat2 = obtener_identificador_lon_lat(punto)
            if (lon2, lat2) == (lon1, lat1):
                encuentro = f"{lon1}_{lat1}"
                if mp.contains(mapa, encuentro):
                    gr.addEdge(grafo, encuentro, punto, 0)
                    gr.addEdge(grafo, punto, encuentro, 0)
                else:
                    mp.put(mapa,encuentro, encuentro)
                    mp.put(mapa_positions,encuentro, encuentro)
                    gr.insertVertex(grafo, encuentro)
                    gr.addEdge(grafo, encuentro, anterior, 0)
                    gr.addEdge(grafo, anterior, encuentro, 0)
                    gr.addEdge(grafo, encuentro, punto, 0)
                    gr.addEdge(grafo, punto, encuentro, 0)
            else:
                encuentro = f"{lon2}_{lat2}"


        else:
            iden, lon2, lat2 = obtener_identificador_lon_lat(punto)
        
        anterior = punto

    control["moves"] = grafo
    control["positions"] = mapa_positions
    control["encuentros"] = mapa
    
    return control, gr.numVertices(grafo), gr.numEdges(grafo), lista



    

    
    
    

# Funciones para creacion de datos
def haversine(lon1, lat1, lon2, lat2):
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371
    return round((c * r), 3)

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

def obtener_identificador_lon_lat(txt):
    txt= txt.split("_")
    if len(txt)== 5:
        valor= txt[2]+"_"+txt[3]+"_"+txt[4]
    elif len(txt)== 4:
        valor= txt[2]+"_"+txt[3]
    else:
        valor=0
        
    return valor, txt[0], txt[1]

def convertir_lon_lat(lon, lat):
    lon= lon.replace("m", "-")
    lon= lon.replace("p", ".")
    lat= lat.replace("m", "-")
    lat= lat.replace("p", ".")
    
    return float(lon), float(lat)

    
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

def obtener_lista_mapa(mapa):

    return mp.keySet(mapa)

def cinco_prim_ult(data_structs):    
    if lt.size(data_structs)<=6:
        return data_structs
    else:
        data_structs2=lt.newList(datastructure="ARRAY_LIST")
        primer = lt.firstElement(data_structs)
        segundo= lt.getElement(data_structs, 2)
        tercero= lt.getElement(data_structs, 3)
        cuarto= lt.getElement(data_structs, 4)
        quinto= lt.getElement(data_structs, 5)
        
        ultimo= lt.lastElement(data_structs)
        penultimo=lt.getElement(data_structs, -2)
        antepenultimo=lt.getElement(data_structs, -3)
        cuartoultimo=lt.getElement(data_structs, -4)
        quintoultimo=lt.getElement(data_structs, -5)
        
        
        lt.addLast(data_structs2, primer)
        lt.addLast(data_structs2, segundo)
        lt.addLast(data_structs2, tercero)
        lt.addLast(data_structs2, cuarto)
        lt.addLast(data_structs2, quinto)
        
        lt.addLast(data_structs2, quintoultimo)
        lt.addLast(data_structs2, cuartoultimo)
        lt.addLast(data_structs2, antepenultimo)
        lt.addLast(data_structs2, penultimo)
        lt.addLast(data_structs2, ultimo)
        
        
        return data_structs2
    
def prim_ult(data_structs): 
    if lt.size(data_structs)<=2:
        return data_structs
    else:
        data_structs2=lt.newList(datastructure="ARRAY_LIST")
        primer = lt.firstElement(data_structs)
        ultimo= lt.lastElement(data_structs)
        lt.addLast(data_structs2, primer)
        lt.addLast(data_structs2, ultimo)
        return data_structs2
    
def obtener_lat(data_structs):
    data_structs2=lt.newList(datastructure="ARRAY_LIST")
    for data in lt.iterator(data_structs):
        iden, lon, lat= obtener_identificador_lon_lat(data)
        lon, lat= convertir_lon_lat(lon, lat)
        lt.addLast(data_structs2, lat)
        
    return data_structs2

def obtener_lon(data_structs):
    data_structs2=lt.newList(datastructure="ARRAY_LIST")
    for data in lt.iterator(data_structs):
        iden, lon, lat= obtener_identificador_lon_lat(data)
        lon, lat= convertir_lon_lat(lon, lat)
        lt.addLast(data_structs2, lon)
    return data_structs2

def cola_carga_de_datos(control, data_structs):
    queue= qu.newQueue()
    grafo=control["moves"]
    for data in lt.iterator(data_structs):
        iden, lon, lat= obtener_identificador_lon_lat(data)
        lon, lat= convertir_lon_lat(lon, lat)
        
        node_id= data
        individual_id= ""
        if iden== 0:
            lista= gr.adjacents(grafo, data)
            for data in lista:
                individual_id= individual_id+","+ data
        else:
            individual_id= data
            
        individual_id= individual_id.strip(",")
        
        adjacent_nodes= gr.outdegree(grafo, data)
        
        valor= str(lon)+";"+str(lat)+";"+data+";"+individual_id+";"+str(adjacent_nodes)
        
        qu.enqueue(queue, valor)
    
    return queue
        

def req_1(data_structs, inc, fin):
    """
    Función que soluciona el requerimiento 1
    """
    # TODO: Realizar el requerimiento 1
    recorrido= dfs.DepthFirstSearch(data_structs, inc)
    puntos_en= 0
    suma_arc=0
    if dfs.hasPathTo(recorrido, fin):
        path = dfs.pathTo(recorrido, fin)
        size= st.size(path)
        lista=lt.newList(datastructure="ARRAY_LIST")
        while not st.isEmpty(path):
            vertex = st.pop(path)
            txt= vertex.split("_")
            if len(txt)==2:
                puntos_en+=1
            data= crear_datos_req1(data_structs, vertex)
            if st.size(path) != size-1:
                if st.isEmpty(path):
                    arco="Unknown"
                    vertice= "Unknown"
                    data= data, vertice, arco
                else:
                    anterior= lt.lastElement(lista)
                    arco= gr.getEdge(data_structs, anterior[2], data[2])
                    suma_arc+= float(arco["weight"])
                    anterior= anterior, vertex, arco["weight"]
                    lt.removeLast(lista)
                    lt.addLast(lista, anterior)
            lt.addLast(lista, data)
        return lista, size, puntos_en, suma_arc
            
    else: 
        return False

        
def crear_datos_req1(grafo, vertex):
    iden, lon, lat= obtener_identificador_lon_lat(vertex)
    lon, lat= convertir_lon_lat(lon, lat)
    individual_id= ""
    if iden== 0:
            lista= gr.adjacents(grafo, vertex)
            
            for data in lt.iterator(lista):
                iden2, lon2, lon2 = obtener_identificador_lon_lat(data)
                individual_id= individual_id+","+ iden2
            individual_id= individual_id.strip(",")
    else:
        
        individual_id= iden
    individual_count= individual_id.split(",")
    individual_count=len(individual_count)
    
    return lon, lat, vertex, individual_id, individual_count
           

def req_2(data_structs, inc, fin):
    """
    Función que soluciona el requerimiento 2
    """
    # TODO: Realizar el requerimiento 2
    recorrido= bfs.BreadhtFisrtSearch(data_structs, inc)
    puntos_en= 0
    suma_arc=0
    if bfs.hasPathTo(recorrido, fin):
        path = bfs.pathTo(recorrido, fin)
        size= st.size(path)
        lista=lt.newList(datastructure="ARRAY_LIST")
        while not st.isEmpty(path):
            vertex = st.pop(path)
            txt= vertex.split("_")
            if len(txt)==2:
                puntos_en+=1
            data= crear_datos_req1(data_structs, vertex)
            if st.size(path) != size-1:
                if st.isEmpty(path):
                    arco="Unknown"
                    vertice= "Unknown"
                    data= data, vertice, arco
                else:
                    anterior= lt.lastElement(lista)
                    arco= gr.getEdge(data_structs, anterior[2], data[2])
                    suma_arc+= float(arco["weight"])
                    anterior= anterior, vertex, arco["weight"]
                    lt.removeLast(lista)
                    lt.addLast(lista, anterior)
            lt.addLast(lista, data)
        return lista, size, puntos_en, suma_arc
            
    else: 
        return False


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


def req_5(data_structs, puntos, kil, inc):
    """
    Función que soluciona el requerimiento 5
    """
    # TODO: Realizar el requerimiento 5
    grafo= data_structs["moves"]
    mapa_postions= data_structs["positions"]
    lista_positions= mp.keySet(mapa_postions)
    kil= (float(kil)/2)
    recorridos= djk.Dijkstra(grafo, inc)
    encuentros=om.newMap("BST",
                      compare_arbol_caso)
    for encuentro in lt.iterator(lista_positions):
        costo=djk.distTo(recorridos, encuentro)
        if costo<= kil:
            om.put(encuentros, costo, encuentro)
            
    rutas= om.size(encuentros)
    if rutas!=0:
        valor= obtener_recorrido_max(recorridos, encuentros, puntos)
        if valor!= False:
            recorrido_mayor, distancia, min_pun= valor
            lista_vertices=lt.newList(datastructure="ARRAY_LIST")
            lista_animales=lt.newList(datastructure="ARRAY_LIST")
            size= st.size(recorrido_mayor)
            while not st.isEmpty(recorrido_mayor):
                vertex = st.pop(recorrido_mayor)
                vertex= vertex["vertexA"]
                lt.addLast(lista_vertices, vertex)
            
            
         
            lista_vertices= sort(lista_vertices, 2)
            lista_ver_2= lista_vertices
            for vertex in lt.iterator(lista_ver_2):
                txt= vertex.split("_")
                if len(txt)==2:
                    animals= gr.outdegree(grafo, vertex)
                else:
                    animals=1
                lt.addLast(lista_animales, animals)
            respuesta= size, distancia, lista_vertices, lista_animales
            return rutas, min_pun, distancia*2, respuesta
        
    else: 
        return False
    
    return False
        
def obtener_recorrido_max(recorridos, mapa, valor):
    lista= om.keySet(mapa)
    while lt.size(lista) != 0:
        distancia_max= om.maxKey(mapa)
        entry= om.get(mapa, distancia_max)
        value= me.getValue(entry)
        path= djk.pathTo(recorridos, value)
        puntos= st.size(path)
        if puntos>= int(valor):
            return path, distancia_max, puntos
        else:
            om.deleteMax(mapa)
        lt.removeLast(lista)
    return False

def contar_puntos_encuentros(path):
    puntos_en=0
    while not st.isEmpty(path):
            vertex = st.pop(path)
            txt= vertex.split("_")
            if len(txt)==2:
                puntos_en+=1
    return puntos_en
            
    

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

def sort(data_structs, num):
    """
    Función encargada de ordenar la lista con los datos
    """
    #TODO: Crear función de ordenamiento
    if num ==1:
        data_structs= merg.sort(data_structs, sort_event)
    elif num ==2:
        data_structs= merg.sort(data_structs, sort_lon_lat)        
    elif num == 3:
        data_structs= merg.sort(data_structs, sort_latitud)
    return data_structs


def sort_event(data1,data2):
    if str(data1["individual-local-identifier"]) < str(data2["individual-local-identifier"]):
        return True
    elif str(data1["individual-local-identifier"]) == str(data2["individual-local-identifier"]):
        
        return data1["timestamp"] < data2["timestamp"]
    else:
        return False
    
def sort_lon_lat(data1,data2):
        
    return data1< data2


    
def sort_latitud(data1,data2):
    iden, lon1, lat1 = obtener_identificador_lon_lat(data1)
    iden, lon2, lat2 = obtener_identificador_lon_lat(data2)
    return lat1<lat2

def compare_arbol_caso(data_1, data_2):
    if data_1 > data_2:
        return 1
    elif data_1 < data_2:
        return -1
    else:
        return 0
