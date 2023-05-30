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
import folium

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
    control["wolfs"]= mp.newMap(200,
                                        maptype='PROBING',
                                        loadfactor=0.5,
                                        cmpfunction=compare_map)
    
    control["positions"]= mp.newMap(3,
                                    maptype='PROBING',
                                    loadfactor=0.5,
                                    cmpfunction=compare_map)
    
    control["encuentros"]= mp.newMap(660,
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
        
        gr.insertVertex(grafo, punto)
        mp.put(mapa, punto, evento)
        
        if anterior is not None and individual_id == anterior["individual-local-identifier"] + "_" + anterior["tag-local-identifier"]:
            punto_ant = crear_identificador(anterior)
            if gr.getEdge(grafo, punto_ant, punto)== None and (punto_ant != punto):
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
    
    lista = sort(mp.keySet(mapa_positions), 2)
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
    lat= f"{float(data['location-lat']):.3f}"
    lat= str(lat)
    lat= lat.replace("-", "m")
    lat= lat.replace(".", "p")
    
    lon=f"{float(data['location-long']):.3f}"
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
    if lt.size(data_structs)<=10:
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
    
def tres_prim_ult(data_structs):    
    if lt.size(data_structs)<=6:
        return data_structs
    else:
        data_structs2=lt.newList(datastructure="ARRAY_LIST")
        primer = lt.firstElement(data_structs)
        segundo= lt.getElement(data_structs, 2)
        tercero= lt.getElement(data_structs, 3)
        
        ultimo= lt.lastElement(data_structs)
        penultimo=lt.getElement(data_structs, -2)
        antepenultimo=lt.getElement(data_structs, -3)
        
        
        lt.addLast(data_structs2, primer)
        lt.addLast(data_structs2, segundo)
        lt.addLast(data_structs2, tercero)
        
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
        if True:
            iterator = lt.iterator(lista)
            ln_i = lt.firstElement(lista)[0][0]
            lt_i = lt.firstElement(lista)[0][1]
            m = folium.Map(location=[lt_i, ln_i], zoom_start=12)
            trail = []
            for i in iterator:
                if type(i[1]) != float: 
                    trail.append([i[0][1], i[0][0]])
            folium.PolyLine(trail).add_to(m)
            output_file = "req1.html"
            m.save(output_file)
            
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
        if True:
            iterator = lt.iterator(lista)
            ln_i = lt.firstElement(lista)[0][0]
            lt_i = lt.firstElement(lista)[0][1]
            m = folium.Map(location=[lt_i, ln_i], zoom_start=12)
            trail = []
            for i in iterator:
                if type(i[1]) != float: 
                    trail.append([i[0][1], i[0][0]])
            folium.PolyLine(trail).add_to(m)
            output_file = "req2.html"
            m.save(output_file)
        return lista, size, puntos_en, suma_arc
            
    else: 
        return False


def req_3(control):
    """
    Función que soluciona el requerimiento 3
    """
    
    #se ejecuta kosaraju y se almacena los IDSCC o los identificadores de las manadas
    KosarajuData = scc.KosarajuSCC(control['moves']) 
    idscc = KosarajuData['idscc']
    IDkeys = mp.keySet(idscc)
    respuesta= datos_kosaraju(control,idscc,KosarajuData)
  
    
def datos_kosaraju(control,idscc,IDkeys):
    """funcion para cargar y organizar los datos generados por el alogritmo de KosarajuSCC

    Args:
        control: el modelo
        idscc: tabla con las idscc generadas por el alogritmo de kosaraju
        IDkeys: La llaves de la tabla idscc
    """
    infoManadas = mp.newMap(maptype="PROBING")
    for IDkey in lt.iterator(IDkeys):
        manada = mp.get(idscc,IDkey)
        lon= getLongitud(IDkey)
        lat= getLatitud(IDkey)
        
        if not mp.contains(infoManadas, manada):
            info = mp.newMap(maptype="PROBING")
            nodeIds = lt.newList(datastructure="ARRAY_LIST")
            Wolfs = lt.newList(datastructure="ARRAY_LIST")
            wolfDetails = lt.newList(datastructure="ARRAY_LIST")
            info = infoManada(control, info, nodeIds, wolfDetails, lat, lat, lon, lon, IDkey, Wolfs)  
            mp.put(infoManadas, manada, info)
        else:
            info = mp.get(infoManadas, manada)['value']
            nodeIds = mp.get(info, "Nodes Ids")['value']
            Wolfs = mp.get(info, "Wolfs")['value']
            wolfDetails = mp.get(info, "Wolfs Details")['value']
            min_lat = min(float(mp.get(info, "min-lat")['value']), lat)
            max_lat = max(float(mp.get(info, "max-lat")['value']), lat)
            min_long = min(float(mp.get(info, "min-long")['value']), lon)
            max_long = max(float(mp.get(info, "max-long")['value']), lon)
            info = infoManada(control, info, nodeIds,wolfDetails,min_lat, max_lat, min_long, max_long, IDkey, Wolfs)
    
    return infoManadas
        


def req_4(data, ori_lon, ori_lat, des_lon, des_lat):
    """
    Función que soluciona el requerimiento 4
    """
    # TODO: Realizar el requerimiento 4
    cont = 1
    lista = lt.newList(datastructure="ARRAY_LIST")
    lista2 = lt.newList(datastructure="ARRAY_LIST")
    lobos = lt.newList(datastructure="ARRAY_LIST")
    lista_mapa = lt.newList(datastructure="ARRAY_LIST")
    lista_enc = lt.newList(datastructure="ARRAY_LIST")
    mapa_postions = data["positions"]
    positions = data["encuentros"]
    lista_positions= mp.keySet(positions)
    p1 = lt.firstElement(lista_positions)
    _, dist1_lon, dist1_lat = obtener_identificador_lon_lat(p1)
    dist1_lon, dist1_lat = convertir_lon_lat(dist1_lon, dist1_lat)
    dist_ori = haversine(dist1_lon, dist1_lat, ori_lon, ori_lat)
    dist_des = haversine(dist1_lon, dist1_lat, des_lon, des_lat)
    ori = p1
    des = p1
    for pos in lt.iterator(lista_positions):
        _, lon, lat = obtener_identificador_lon_lat(pos)
        lon, lat = convertir_lon_lat(lon, lat)
        dist_ori_pos = haversine(lon, lat, ori_lon, ori_lat)
        dist_des_pos = haversine(lon, lat, des_lon, des_lat)
        if dist_ori_pos < dist_ori:
            dist_ori = dist_ori_pos
            ori = pos
        if dist_des_pos < dist_des:
            dist_des = dist_des_pos
            des = pos
    grafo= data["moves"]        
    rec = djk.Dijkstra(grafo, ori)
    if djk.hasPathTo(rec, des):
        costo = djk.distTo(rec, des)
        camino = djk.pathTo(rec, des)
        num_nodos = st.size(camino)
        for _ in range(num_nodos):
            ele = st.pop(camino)
            vertice = ele["vertexA"]
            lt.addLast(lista, vertice)
        vertice = ele["vertexB"]
        lt.addLast(lista, vertice)
        for i in lt.iterator(lista):
            id, a, b = obtener_identificador_lon_lat(i)
            if id == 0: 
                dato = crear_datos_req4(grafo, i)
                lt.addLast(lista_enc, i)
            else:
                if not lt.isPresent(lobos, str(id)):
                    lt.addLast(lobos, str(id))
                dato = crear_datos_req4(grafo, i)    
            lt.addLast(lista_mapa, dato)
        num_arcos = num_nodos
        for c in lt.iterator(lista_enc):
            cont += 1
            dato = crear_datos_req4(grafo, c)
            if c != lt.lastElement(lista):
                dist_nxt = djk.distTo(rec, lt.getElement(lista_enc, cont)) - djk.distTo(rec, c)
            else:
                dist_nxt = 0
            dato.append(dist_nxt)
            lt.addLast(lista2, dato)

        if True:
            iterator = lt.iterator(lista_mapa)
            ln_i = lt.firstElement(lista_mapa)[0]
            lt_i = lt.firstElement(lista_mapa)[1]
            m = folium.Map(location=[lt_i, ln_i], zoom_start=12)
            trail = []
            for i in iterator:
                trail.append([i[1], i[0]])
            folium.PolyLine(trail).add_to(m)
            output_file = "req4.html"
            m.save(output_file)


    else:
        costo = "Desconocido"
        num_nodos = "Desconocido"
        num_lobos = "Desconocido"
        num_arcos = "Desconocido"

    lista_final = tres_prim_ult(lista2)
    num_lobos = lt.size(lobos)
    return dist_ori, dist_des, costo, lt.size(lista2), num_lobos, num_arcos, lista_final

def crear_datos_req4(grafo, vertex):
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
    return [lon, lat, vertex, individual_id, individual_count]

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
            
    

def req_6(control, inc, fin, gen):
    """
    Función que soluciona el requerimiento 6
    """
    #Crea un arbol y una lista con los eventos en el rango ingresado, y la ordena por lobo y fecha
    arbol= crear_arbol_fechas(control["positions"])
    lista_rango= lista_eventos_rango(arbol, inc, fin)
    lista_rango= sort(lista_rango, 1)
    
    #Obtener los lobos por género y crear una lista con ellos
    wolfs= mp.valueSet(control["wolfs"])
    wolfs_gen= lt.newList(datastructure="ARRAY_LIST")
    for wolf in lt.iterator(wolfs):
        if wolf["animal-sex"].lower()== gen:
            lt.addLast(wolfs_gen, wolf)
            
    # Crea un mapa con listas de eventos de cada lobo
            
    mapa_wolf= crear_tabla_con_lobos_eventos(wolfs_gen, lista_rango)
    
    # Obtiene los mayores y menores recorridos a partir del mapa y el grafo
    mayor= -999999
    menor= 999999999
    lista_may=None
    lista_men=None
    lista_wolfs= mp.valueSet(mapa_wolf)
    iden_may=""
    iden_men=""
    
    grafo= gr.newGraph(datastructure= "ADJ_LIST",
                                      directed= False,
                                      size=300000,
                                      cmpfunction=compareStopIds)
    
    for eventos in lt.iterator(lista_wolfs):
        grafo= agregar_al_grafo(grafo, eventos)
        peso= obtener_distancia_total(grafo, eventos)
        if peso > mayor:
            mayor=peso
            lista_may= eventos
            iden_may= lt.firstElement(eventos)
            iden_may=iden_may["individual-local-identifier"]+"_"+iden_may["tag-local-identifier"]
        elif peso < menor:
            lista_men= eventos
            menor= peso
            iden_men= lt.firstElement(eventos)
            iden_men=iden_men["individual-local-identifier"]+"_"+iden_men["tag-local-identifier"]
            
    
    wolf_may= mp.get(control["wolfs"], iden_may)
    wolf_may= me.getValue(wolf_may)
    nodos_may= lt.size(lista_may)
    nodos_men= lt.size(lista_men)
    lista_may= tres_prim_ult(lista_may)
    lista_men= tres_prim_ult(lista_men)
    
    wolf_men= mp.get(control["wolfs"], iden_men)
    wolf_men= me.getValue(wolf_men)
    
    
    return mayor, iden_may, wolf_may, lista_may, nodos_may, menor, iden_men, wolf_men, lista_men, nodos_men, gr.numEdges(grafo), gr.numVertices(grafo)
    
def agregar_al_grafo(grafo, lista):
    anterior=None
    for data in lt.iterator(lista):
        punto= crear_identificador(data)
        gr.insertVertex(grafo, punto)
        if anterior is not None:
            punto_ant= crear_identificador(anterior)
            lon1 = round(float(anterior["location-long"]), 3)
            lat1 = round(float(anterior["location-lat"]), 3)
            lon2 = round(float(data["location-long"]), 3)
            lat2 = round(float(data["location-lat"]), 3)

            peso = haversine(lon1, lat1, lon2, lat2)
            gr.addEdge(grafo, punto_ant, punto, peso)
        
        anterior= data
    
    return grafo
    
    
def obtener_distancia_total(grafo, lista):
    peso=0
    first= lt.firstElement(lista)
    punto= crear_identificador(first)
    last= lt.lastElement(lista)
    punto_last= crear_identificador(last)
    search= djk.Dijkstra(grafo, punto)
    peso= djk.distTo(search, punto_last)
    
    return peso



def crear_tabla_con_lobos_eventos(wolfs_gen, lista_rango):
    mapa_event= mp.newMap(200,
                                        maptype='PROBING',
                                        loadfactor=0.5,
                                        cmpfunction=compare_map)
    lista_eventos= lt.newList(datastructure="ARRAY_LIST")
    first= lt.firstElement(lista_rango)
    anterior= None
    for evento in lt.iterator(lista_rango):
        individual_id=evento["individual-local-identifier"]+"_"+evento["tag-local-identifier"]
        if individual_id == anterior:
            entry= mp.get(mapa_event, individual_id)
            value= me.getValue(entry)
            lt.addLast(value, evento)
            mp.put(mapa_event, individual_id, value)
        else:
            lista_eventos= lt.newList(datastructure="ARRAY_LIST")
            lt.addLast(lista_eventos, evento)
            mp.put(mapa_event, individual_id, lista_eventos)
            
        anterior= individual_id
    
    mapa_wolfs= mp.newMap(200,
                                        maptype='PROBING',
                                        loadfactor=0.5,
                                        cmpfunction=compare_map) 
    
    for wolf in lt.iterator(wolfs_gen):
        individual_id=wolf["animal-id"]+"_"+wolf["tag-id"]
        entry= mp.get(mapa_event, individual_id)
        if entry != None:
            value= me.getValue(entry)
            mp.put(mapa_wolfs, individual_id, value)
        
    return mapa_wolfs
    
    
    
    
    
    
def lista_eventos_rango(arbol, inc, fin):
    lista_rango= om.values(arbol, inc, fin)
    lista_eventos=lt.newList(datastructure="ARRAY_LIST")
    for data in lt.iterator(lista_rango):
        for evento in lt.iterator(data):
            lt.addLast(lista_eventos, evento)
    
    return lista_eventos

def crear_arbol_fechas(mapa):
    lista= crear_lista_movimientos(mapa)
    lista= sort(lista, 4)
    arbol=om.newMap("RBT",
                      compare_arbol_caso)
    
    lista_valores= lt.newList(datastructure="ARRAY_LIST")
    first= lt.firstElement(lista)
    fecha=first["timestamp"]
    for data in lt.iterator(lista):
        fecha_data=data["timestamp"]
        if data== first:
            lt.addLast(lista_valores, data)
            om.put(arbol, fecha_data, lista_valores)
        elif fecha_data== fecha:
            entry= om.get(arbol, fecha_data)
            value= me.getValue(entry)
            lt.addLast(value, data)
            om.put(arbol, fecha, value)
        else:
            lista_valores= lt.newList(datastructure="ARRAY_LIST")
            lt.addLast(lista_valores, data)
            om.put(arbol, fecha_data, lista_valores)
            fecha= fecha_data

    return arbol
    


def crear_lista_movimientos(mapa):
    lista=lt.newList(datastructure="ARRAY_LIST")
    lista_posiciones= mp.valueSet(mapa)
    for data in lt.iterator(lista_posiciones):
        if type(data) != str:
            lt.addLast(lista, data)
            
    return lista



def req_7(control, inc, fin, tem_min, tem_max):
    """
    Función que soluciona el requerimiento 7
    """
    #Crea un arbol y una lista con los eventos en el rango ingresado, y la ordena por lobo y fecha
    arbol= crear_arbol_fechas(control["positions"])
    lista_rango= lista_eventos_rango(arbol, inc, fin)
    lista_rango= sort(lista_rango, 1)
    
    #Crear lista de eventos apartir de la temperatura
    lista_eventos=lt.newList(datastructure="ARRAY_LIST")
    for evento in lt.iterator(lista_rango):
        if float(evento["external-temperature"])> float(tem_min) and  float(evento["external-temperature"])<float(tem_max):
            lt.addLast(lista_eventos, evento)
            
    lista_eventos= sort(lista_eventos, 1)
    grafo= crear_grafo_manadas(lista_eventos)
    search= scc.KosarajuSCC(grafo)
    
    
    
    

def crear_grafo_manadas(lista):
    lista_puntos=lt.newList(datastructure="ARRAY_LIST")
    anterior= None
    grafo= gr.newGraph(datastructure= "ADJ_LIST",
                                      directed= False,
                                      size=300000,
                                      cmpfunction=compareStopIds)
    search= scc.KosarajuSCC(grafo)
    print(search)
    anterior = None

    for evento in lt.iterator(lista):
        punto = crear_identificador(evento)
        individual_id = evento["individual-local-identifier"] + "_" + evento["tag-local-identifier"]
        
        gr.insertVertex(grafo, punto)
        
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
        
    lista_puntos= sort(lista_puntos, 2)
    anterior = None

    for punto in lt.iterator(lista_puntos):
        if anterior is not None:
            iden1, lon1, lat1 = obtener_identificador_lon_lat(anterior)
            iden2, lon2, lat2 = obtener_identificador_lon_lat(punto)
            if (lon2, lat2) == (lon1, lat1):
                encuentro = f"{lon1}_{lat1}"
                if gr.containsVertex(grafo, encuentro):
                    gr.addEdge(grafo, encuentro, punto, 0)
                    gr.addEdge(grafo, punto, encuentro, 0)
                else:
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
        
    return grafo
        

def req_8(data_structs):
    """
    Función que soluciona el requerimiento 8
    """
    # TODO: Realizar el requerimiento 8
    pass


# Funciones utilizadas para comparar elementos dentro de una lista

def infoManada(control, info, nodeIds, wolfDetails, min_lat, max_lat, min_long, max_long, point, Wolfs):
    """"
    Actualiza los campos de información de una manada
    """
    lt.addLast(nodeIds, point)
    mp.put(info, "Nodes Ids", nodeIds)
    mp.put(info, "SCC Size", lt.size(nodeIds))
    mp.put(info, "min-lat", min_lat)
    mp.put(info, "max-lat", max_lat)
    mp.put(info, "min-long", min_long)
    mp.put(info, "max-long", max_long)
    if (mp.contains(control['locations'], point)):
        individualId = mp.get(control['locations'], point)['value']
        individualId = individualId['individual-id']
        if (mp.contains(control["wolfs"], individualId)) and (not lt.isPresent(Wolfs, individualId)):
            lt.addLast(wolfDetails, getWolfsDetails(control, individualId)) 
            lt.addLast(Wolfs, individualId)
    mp.put(info, "Wolfs", Wolfs)
    mp.put(info, "Wolfs Count", lt.size(wolfDetails))
    mp.put(info, "Wolfs Details", wolfDetails)
    return info


def getWolfsDetails(control, individualId):
    """
    Da el formato de información de lobos requeridos 
    en el requerimiento 3
    """
    wolf = {
        'individual-id': None,
        'animal-sex': None,
        'animal-life-stage': None,
        'study-site': None,
        'deployment-comments': None,}

    wolfInfo = mp.get(control['wolfs'], individualId)['value']
    wolf['individual-id'] = individualId
    wolf['animal-sex'] = wolfInfo['animal-sex'] if wolfInfo['animal-sex'] != "" else "Unknown"
    wolf['animal-life-stage'] = wolfInfo['animal-life-stage'] if wolfInfo['animal-life-stage'] != "" else "Unknown"
    wolf['study-site'] = wolfInfo['study-site'] if wolfInfo['study-site'] != "" else "Unknown"
    wolf['deployment-comments'] = wolfInfo['deployment-comments'] if wolfInfo['deployment-comments'] != "" else "Unknown"
    
    return wolf


def getLongitud(ID):
    """
        Retorna la latitud de un IDkey
    """
    return float(ID.split('_')[0].replace('m', '-').replace('p', '.'))

def getLatitud(ID):
    """
        Retorna la latitud de un IDkey
    """
    return float(ID.split('_')[1].replace('m', '-').replace('p', '.'))

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
    elif num == 4:
        data_structs= merg.sort(data_structs, sort_fecha)
    return data_structs


def sort_event(data1,data2):
    if str(data1["individual-local-identifier"]) < str(data2["individual-local-identifier"]):
        return True
    elif str(data1["individual-local-identifier"]) == str(data2["individual-local-identifier"]):
        
        if str(data1["tag-local-identifier"]) < str(data2["tag-local-identifier"]):
            return True
        elif str(data1["tag-local-identifier"]) == str(data2["tag-local-identifier"]):
            return data1["timestamp"] < data2["timestamp"]
        else: 
            return False
    else:
        return False
    
def sort_fecha(data1,data2):
    return data1["timestamp"] < data2["timestamp"]
    
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
