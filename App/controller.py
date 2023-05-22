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
 """

import config as cf
import model
import time
import csv
import tracemalloc

"""
El controlador se encarga de mediar entre la vista y el modelo.
"""


def new_controller():
    """
    Crea una instancia del modelo
    """
    #TODO OK: Llamar la función del modelo que crea las estructuras de datos
    control={"wolfs": None,
            "positions": None, 
            "moves": None,
            "encuentros": None}
    control =model.new_data_structs(control)
    return control


# Funciones para la carga de datos

def load_data(control, filename):
    """
    Carga los datos del reto
    """
    # TODO: Realizar la carga de datos
    start = get_time()
    lista_eventos=model.new_list()
    #carga los nombres de archivo y los asigna a 2 variables
    tracks , wolfs = filename
    
    #itera sobre los datos de los lobos como individuos y se almacena en una tabla "mapa"
    file_1 = cf.data_dir + wolfs
    input_file = csv.DictReader(open(file_1, encoding='utf-8'))
    for data1 in input_file:
        control= model.add_wolfs(control, data1)
    
    #itera sobre los registros de posicion de los lobos y los añade a una lista
    file_2 = cf.data_dir + tracks
    input_file_2 = csv.DictReader(open(file_2, encoding='utf-8'))
    for data2 in input_file_2:
        lista_eventos= model.add_list_evento(data2,lista_eventos)
    #se ordena la lista por manada, por lobo y por fecha
    lista_eventos = model.sort(lista_eventos, 1)
    num_event= model.data_size(lista_eventos)


    control, vertices_in, arcos_in= model.load_moves(control,lista_eventos)
    
    
    
    control, vertices, arcos = model.agregar_encuentros(control)
    
    lista_eventos= model.sort(lista_eventos, 3)
    lista_lat= model.prim_ult(lista_eventos)
    
    lista_eventos= model.sort(lista_eventos, 4)
    lista_lon= model.prim_ult(lista_eventos)
    
    
    lista_vertices= model.obtener_lista_vertices(control["moves"])
    lista_vertices= model.sort(lista_vertices, 2)
    lista_vertices= model.cinco_prim_ult(lista_vertices)
    
    queue= model.cola_carga_de_datos(control, lista_vertices)
    
    
    
    
    
    end= get_time()
    deltatime = delta_time(start, end)
    print(deltatime)
    return control, num_event, vertices_in, arcos_in, vertices, arcos, lista_lat, lista_lon, queue


# Funciones de ordenamiento

def sort(control):
    """
    Ordena los datos del modelo
    """
    #TODO: Llamar la función del modelo para ordenar los datos
    pass


# Funciones de consulta sobre el catálogo

def get_data(control, id):
    """
    Retorna un dato por su ID.
    """
    #TODO: Llamar la función del modelo para obtener un dato
    pass


def req_1(control):
    """
    Retorna el resultado del requerimiento 1
    """
    # TODO: Modificar el requerimiento 1
    pass


def req_2(control):
    """
    Retorna el resultado del requerimiento 2
    """
    # TODO: Modificar el requerimiento 2
    pass


def req_3(control):
    """
    Retorna el resultado del requerimiento 3
    """
    # TODO: Modificar el requerimiento 3
    pass


def req_4(control):
    """
    Retorna el resultado del requerimiento 4
    """
    # TODO: Modificar el requerimiento 4
    pass


def req_5(control):
    """
    Retorna el resultado del requerimiento 5
    """
    # TODO: Modificar el requerimiento 5
    pass

def req_6(control):
    """
    Retorna el resultado del requerimiento 6
    """
    # TODO: Modificar el requerimiento 6
    pass


def req_7(control):
    """
    Retorna el resultado del requerimiento 7
    """
    # TODO: Modificar el requerimiento 7
    pass


def req_8(control):
    """
    Retorna el resultado del requerimiento 8
    """
    # TODO: Modificar el requerimiento 8
    pass


# Funciones para medir tiempos de ejecucion

def get_time():
    """
    devuelve el instante tiempo de procesamiento en milisegundos
    """
    return float(time.perf_counter()*1000)


def delta_time(start, end):
    """
    devuelve la diferencia entre tiempos de procesamiento muestreados
    """
    elapsed = float(end - start)
    return elapsed

def get_memory():
    """
    toma una muestra de la memoria alocada en instante de tiempo
    """
    return tracemalloc.take_snapshot()


def delta_memory(stop_memory, start_memory):
    """
    calcula la diferencia en memoria alocada del programa entre dos
    instantes de tiempo y devuelve el resultado en bytes (ej.: 2100.0 B)
    """
    memory_diff = stop_memory.compare_to(start_memory, "filename")
    delta_memory = 0.0

    # suma de las diferencias en uso de memoria
    for stat in memory_diff:
        delta_memory = delta_memory + stat.size_diff
    # de Byte -> kByte
    delta_memory = delta_memory/1024.0
    return delta_memory
