# -*- coding: utf-8 -*-
"""
Created on Sat Feb  26 19:57:01 2022

@author: Regojo Yan
"""
from matplotlib import pyplot as plt
import numpy as np
from PIL import Image

def Show(array):
    array = np.array(array) # Conversion tableau python à np ndarray
    array = array.astype(np.uint8) # Conversion 0 255 (8bit)
    im = Image.fromarray(array) # Image depuis le tableau
    im.show()

import os
def Open(filename):
    path = os.path.realpath(filename) # Absolute path
    im = Image.open(path) # Ouverture du fichier (aucune fermeture)
    array = np.array(im) # Conversion de l'image en np ndarray
    array = array.astype(np.uint8) # Conversion 0 255 (8bit)
    return array, len(array), len(array[0])

import json
def LoadJson(file):
    line = open(file).readline() # Fichier json = 1 seule ligne
    return json.loads(line) # conversion de json en objet python

def SaveAsJson(array, filename):
    path = os.path.realpath(filename) # Absolute path
    json.dump(array, open(path, 'x')) #Ecriture du tableau en language json

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
        print()
        print("temps : "+str(int(end-self.start))+"s")