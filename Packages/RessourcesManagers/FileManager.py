# -*- coding: utf-8 -*-
"""
Created on Sat Feb  26 19:57:01 2022

@author: Regojo Yan
"""
from matplotlib import pyplot as plt
import numpy as np
from PIL import Image

def AsUint8(array):
    array = np.asarray(array)
    return array.astype(np.uint8)

def AsUint16(array):
    array = np.asarray(array, dtype=np.uint16)
    return array.astype(np.uint16)

def ShowImage(array):
    array = np.array(array) # Conversion tableau python à np ndarray
    array = array.astype(np.uint8) # Conversion 0 255 (8bit)
    im = Image.fromarray(array) # Image depuis le tableau
    im.show()

import os
def OpenImage(filename):
    path = os.path.realpath(filename) # Absolute path
    im = Image.open(path) # Ouverture du fichier (aucune fermeture)
    array = np.array(im) # Conversion de l'image en np ndarray
    array = array.astype(np.uint8) # Conversion 0 255 (8bit)
    return array, len(array), len(array[0])

import json
from jsonc_parser.parser import JsoncParser as jsonc # Json avec des commentaires
def LoadJson(file):
    line = open(file).readline() # Fichier json = 1 seule ligne
    return json.loads(line) # conversion de json en objet python

def SaveAsJson(array, filename):
    path = os.path.realpath(filename) # Absolute path
    json.dump(array, open(path, 'x')) #Ecriture du tableau en language json

class Config:
    DATA = None
    def Get(name):
        if Config.DATA == None:
            Config.Load()
        if Config.DATA[name] != None:
            return Config.DATA[name]
        else:
            print("No such setting : "+name)
    
    def Load():
        path = os.path.realpath("settings.jsonc") # Absolute path
        Config.DATA = jsonc.parse_file(path) # Parsing des données dans le fichier jsonc

import time as t
class ProgressBar:
    def __init__(self, title, length = 70, prefix = "", suffix = "", decimals = 1, fill = '#', printEnd = "\r"):
        self.title = title # Le titre du processus ...
        self.start = t.time() # t0
        self.length = length # taille de caractère de la bare de progression
        self.prefix = prefix # sera print avant le titre, avant le pourcentage
        self.suffix = suffix # sera print après le pourcentage
        self.decimals = decimals # Nombre de décimales
        self.fill = fill # Caractère de progression
        self.printEnd = printEnd # sera print après la barre de progression
        self.prefix += self.title

    def SetProgress(self, x):
        percent = ("{0:." + str(self.decimals) + "f}").format(100 * x) # Pourcentage
        filledLength = int(self.length * x) # Nombre de caractères, maximum self.length
        bar = self.fill * filledLength + '-' * (self.length - filledLength) # Barre de caractères
        print(f"\r{self.prefix} [{bar}] {percent}% {self.suffix}", end = self.printEnd)
        self.progress_x = x # On garde une trace de l'avancement

    def Done(self):
        end = t.time()
        print()
        print("temps : "+str(int(end-self.start))+"s")