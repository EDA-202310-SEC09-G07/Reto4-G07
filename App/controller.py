﻿"""
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
    lista_eventos= model.sort(lista_eventos, 1)
    num_event= model.data_size(lista_eventos)


    control, vertices_in, arcos_in= model.load_moves(control,lista_eventos)
    
    
    
    control, vertices, arcos, lista_vertices = model.agregar_encuentros(control)
    
    
    lista_lon= model.prim_ult(lista_vertices)
    lista_lon= model.obtener_lon(lista_lon)
    
    lista_vertices= model.sort(lista_vertices, 3)
    lista_lat= model.prim_ult(lista_vertices)
    lista_lat= model.obtener_lat(lista_lat)
    
    
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


def req_1(control, inc, fin):
    """
    Retorna el resultado del requerimiento 1
    """
    start = get_time()
    valor= model.req_1(control["moves"], inc, fin)
    end= get_time()
    deltatime = delta_time(start, end)
    print(deltatime)
    return valor


def req_2(control, inc, fin):
    """
    Retorna el resultado del requerimiento 2
    """
    start = get_time()
    valor= model.req_2(control["moves"], inc, fin)
    if valor!= False:
        lista, size, puntos_en, suma_arc= valor
        lista= model.cinco_prim_ult(lista)
        valor= lista, size, puntos_en, suma_arc
        end= get_time()
        deltatime = delta_time(start, end)
        print(deltatime)
        return valor
        return valor
    
    
    else:
        end= get_time()
        deltatime = delta_time(start, end)
        print(deltatime)
        return valor
        return False


def req_3(control):
    """
    Retorna el resultado del requerimiento 3
    """
    # TODO: Modificar el requerimiento 3
    start = get_time()
    valor= model.req_3(control)
    end= get_time()
    deltatime = delta_time(start, end)
    print(deltatime)
    return valor


def req_4(data, ori_lon, ori_lat, des_lon, des_lat):
    """
    Retorna el resultado del requerimiento 4
    """
    start = get_time()
    dist_ori, dist_des, costo, num_nodos, num_lobos, num_arcos, lista_final = model.req_4(data, float(ori_lon), float(ori_lat), float(des_lon), float(des_lat))
    end= get_time()
    deltatime = delta_time(start, end)
    print(deltatime)
    return dist_ori, dist_des, costo, num_nodos, num_lobos, num_arcos, lista_final


def req_5(control, puntos, kil, inc):
    """
    Retorna el resultado del requerimiento 5
    """
    # TODO: Modificar el requerimiento 5
    start = get_time()
    valor= model.req_5(control, puntos, kil, inc)
    if valor!= False:
        end= get_time()
        deltatime = delta_time(start, end)
        print(deltatime)
        return valor
    else:
        end= get_time()
        deltatime = delta_time(start, end)
        print(deltatime)
        return False

def req_6(control, inc, fin, gen):
    """
    Retorna el resultado del requerimiento 6
    """
    start = get_time()
    valor= model.req_6(control, inc, fin, gen)
    end= get_time()
    deltatime = delta_time(start, end)
    print(deltatime)
    return valor


def req_7(control, inc, fin, tem_min, tem_max):
    """
    Retorna el resultado del requerimiento 7
    """
    start = get_time()
    value= model.req_7(control, inc, fin, tem_min, tem_max)
    end= get_time()
    deltatime = delta_time(start, end)
    print(deltatime)
    return value


def req_8(bol):
    """
    Retorna el resultado del requerimiento 8
    """
    bol = model.req_8(bol)
    return bol


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
