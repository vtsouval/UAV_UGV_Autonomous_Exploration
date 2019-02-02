#!/usr/bin/env python

import numpy
from utilities import RvizHandler
from math import pow
from utilities import Cffi
from time import time
from numba import jit
from skimage.morphology import skeletonize


class Topology:

    @staticmethod
    def skeletonizationCffi(ogm, origin, resolution, ogml):

        local = numpy.zeros(ogm.shape)
        local = set_ogm(ogm, local, ogm.shape[0], ogm.shape[1])

        #start = time()
        skeleton = Cffi.thinning(local, ogml)
        #print str('SkeletonizationCffi: Thinning in ') + str(time() - start) + str(' seconds.')

        #start = time()
        skeleton = Cffi.prune(skeleton, ogml, 10)
        #print str('SkeletonizationCffi: Prunning in ') + str(time() - start) + str(' seconds.')

        #start = time()
        '''
        viz = set_skeleton_viz(skeleton, numpy.zeros((set_skeleton_viz_number(skeleton, ogm.shape[0], ogm.shape[1]), 2), dtype=numpy.float64),
                               ogm.shape[0], ogm.shape[1], resolution, origin['x'], origin['y'])
        RvizHandler.printMarker(viz, 1, 0, "map", "art_skeletonization_cffi", [0.5, 0, 0, 0.5], 0.05)
        '''
        '''
        viz = []
        for i in range(0, ogm.shape[0]):
            for j in range(0, ogm.shape[1]):
                if skeleton[i][j] == 1:
                    viz.append([i * resolution + origin['x'],j * resolution + origin['y']])

        print str('SkeletonizationCffi: Visualization in ') + str(time() - start) + str(' seconds.')
        RvizHandler.printMarker(viz, 1, 0, "map", "art_skeletonization_cffi", [0.5, 0, 0, 0.5], 0.05)
        '''
        return skeleton

    @staticmethod
    def skeletonization(ogm, origin, resolution, ogml):

        width = ogm.shape[0]
        height = ogm.shape[1]

        useful_ogm = ogm[ ogml['min_x']:ogml['max_x'] , ogml['min_y']:ogml['max_y'] ]
        useful_width = useful_ogm.shape[0]
        useful_height = useful_ogm.shape[1]

        local = numpy.zeros(ogm.shape)
        useful_local = numpy.zeros(useful_ogm.shape)

        for i in range(0, useful_width):
            for j in range(0, useful_height):
                if useful_ogm[i][j] < 49:
                    useful_local[i][j] = 1
      
        skeleton = skeletonize(useful_local)
        skeleton = Topology.pruning(skeleton, 10)
  
        # Padding
        skeleton_final = numpy.zeros(ogm.shape)
        skeleton_final[ ogml['min_x']:ogml['max_x'] , ogml['min_y']:ogml['max_y'] ] = skeleton
        '''
        viz = []
        for i in range(0, width):
            for j in range(0, height):
                if skeleton_final[i][j] == 1:
                    viz.append([i * resolution + origin['x'],j * resolution + origin['y']])
        RvizHandler.printMarker(viz, 1, 0, "map", "art_skeletonization", [0.5, 0, 0, 0.5], 0.05)
        '''

        return skeleton_final

    @staticmethod
    def topologicalNodes(ogm, skeleton, coverage, brush, final_num_of_nodes=25, erase_distance=25, step=5):

        # Initialize Array for Nodes with Max Value of 2000!
        #start = time()
        nodes = numpy.zeros((2000, 2), dtype=numpy.int64) - int(1)
        nodes = set_topology_node(nodes, ogm, skeleton, coverage, brush, ogm.shape[0], ogm.shape[1])
        nodes = (nodes[0:len(numpy.array(numpy.nonzero(nodes[:,1] != -1))[0]), :]).tolist()
        #print str('TopologicalNodes: Initial Nodes Array in is ') + str(len(nodes)) + str(' in ') + str(time() - start) + str(' seconds.')

        # Minimize Number of nodes by Erasing ( Dynamic Erase to Get only 20 Nodes)
        start = time()
        nodes = minimize_number_of_nodes(nodes, final_num_of_nodes, erase_distance, step)
        print str('TopologicalNodes: Final Nodes Array is ') + str(len(nodes)) + str(' in ') + str(time() - start) + str(' seconds.')

        return nodes

    @staticmethod
    def pruning(img, n):

        for k in range(0, n):
            tmp_img = numpy.copy(img)
            for i in range(0, img.shape[0] - 1):
                for j in range(0, img.shape[1] - 1):
                    if img[i][j] == 1:
                        c = 0
                        for ii in range(-1, 2):
                            for jj in range(-1, 2):
                                c = c + img[i + ii][j + jj]
                        if c == 2:
                            tmp_img[i][j] = 0
            img = numpy.copy(tmp_img)
        return img


# Faster Loops in Native C using Numba


@jit(nopython=True)
def set_ogm(ogm,local,width,height):

    for i in range(0, width):
        for j in range(0, height):
            if ogm[i][j] < 49:
                local[i][j] = 1
    return local


@jit(nopython=True)
def set_topology_node(nodes, ogm, skeleton, coverage, brush, width, height):

    counter = 0
    for i in range(1, width - 1):
        for j in range(1, height - 1):
            if ogm[i][j] <= 49 and brush[i][j] > 3 and skeleton[i][j] == 1 and coverage[i][j] != 100:
                c = 0
                for ii in range(-1, 2):
                    for jj in range(-1, 2):
                        c = c + skeleton[i + ii][j + jj]
                if c == 2 or c == 4:
                    nodes[counter][0] = i
                    nodes[counter][1] = j
                    counter += 1
    return nodes


@jit(nopython=True)
def find_number_of_nodes(ogm, skeleton, coverage, brush, width, height):

    counter = 0
    for i in range(1, width - 1):
        for j in range(1, height - 1):

            if ogm[i][j] <= 49 and brush[i][j] > 3 and skeleton[i][j] == 1 and coverage[i][j] != 100:
                c = 0

                for ii in range(-1, 2):
                    for jj in range(-1, 2):
                        c = c + skeleton[i + ii][j + jj]

                if c == 2 or c == 4:
                    counter += 1

    return counter


@jit#(nopython=True)
def minimize_number_of_nodes(nodes, final_num_of_nodes, erase_distance, step):

    # Run Once to Delete Duplicate Nodes
    change = True
    # If  False means Nodes have Distance > erase_distance
    while change:
        # Reset change Flag!
        change = False
        for i in range(0, len(nodes)):
            for j in range(0, len(nodes)):
                # Avoid checking Nodes Distance to Itself
                if i == j:
                    continue
                n1 = nodes[i]
                n2 = nodes[j]
                # Check Nodes Euclidean Distance and Delete it if it Necessary!
                if pow(n1[0] - n2[0], 2) + pow(n1[1] - n2[1], 2) < 25:
                    change = True
                    del nodes[i]
                    # Break from j Loop
                    break
            # Break from i Loop
            if change:
                break

    # Loop Till  Reach the Constant Number of Nodes Required
    while not len(nodes) < final_num_of_nodes:
        change = True
        erase_distance += step
        while change:
            change = False
            for i in range(0, len(nodes)):
                for j in range(0, len(nodes)):
                    if i == j:
                        continue
                    n1 = nodes[i]
                    n2 = nodes[j]
                    if (pow(n1[0] - n2[0], 2) + pow(n1[1] - n2[1], 2)) < erase_distance:
                        change = True
                        del nodes[i]
                        break
                if change:
                    break

    return nodes
