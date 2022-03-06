# from cvxpy import Solution
import pygame
import sys
from arena import Arena
import heapq
from queue import PriorityQueue
import math
import time

class Dijkstra:

    def __init__(self):
        self.dirs = [ [-1, -1],[-1, 0], [-1, 1],
                            [0, -1], [0, 0], [0, 1],
                            [1, -1], [1, 0], [1, 1]] 
        self.recently_closed=[]

    def search(self, arena):
        sol_fnd = False
        open_nodes = arena.open_nodes.copy()
        for _,curr_node in open_nodes.items():
            if curr_node== arena.goal_location:
                arena.goal_location=curr_node
                sol_fnd = True
            for dir in self.dirs:
                x_ = curr_node.x + dir[0]
                y_ = curr_node.y + dir[1]
                node = Arena.Node(x_, y_)
                if (arena.isCollision(x_,y_)):
                    obstacle_node = arena.obstacle_nodes.get((node.x,node.y))
                    if not obstacle_node:
                        arena.obstacle_nodes[(x_, y_)] = node
                    continue

                if(not arena.isValid(node)):
                    continue
                
                # Skip evaluating open nodes:
                open_node_visited = arena.open_nodes.get((node.x,node.y))
                if open_node_visited:
                    # Skip evaluating open nodes' parents:
                    open_node_parent_visited = arena.open_nodes.get((open_node_visited.parent.x,open_node_visited.parent.y))
                    if open_node_parent_visited:
                        continue
                    continue

                # Skip evaluating visited nodes:
                node_visited = arena.nodes.get((node.x,node.y))
                # print(node_visited,node.x,node.y)
                if node_visited:
                    continue
                
                costToCome = curr_node.costToCome + math.sqrt(math.pow(x_-curr_node.x,2)+math.pow(y_-curr_node.y,2))
                if node.costToCome > costToCome:
                    node.parent=curr_node
                    node.costToCome=costToCome
                arena.open_nodes[(node.x,node.y)]=node
            arena.nodes[(curr_node.x, curr_node.y)] = curr_node
            del arena.open_nodes[(curr_node.x, curr_node.y)]
        return sol_fnd, arena

if __name__ == "__main__":
    arena = Arena()
    dijkstra = Dijkstra()
    sol_fnd = False
    while(not sol_fnd): # your main loop
        # get all events
        arena.updateEvents()
        
        #Search Dijsktra
        sol_fnd, arena = dijkstra.search(arena)

        # Update MAP - Pygame display
        arena.drawAll()
    arena.displayResults()
