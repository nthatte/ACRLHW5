#adapted from http://interactivepython.org/courselib/static/pythonds/Graphs/graphintro.html
import vertex
import heapq
import numpy as np

class Graph:
	def __init__(self):
		self.vertList = {}
		self.ptList = []
		self.numVertices = 0

	def addVertex(self,key, pt):
		self.numVertices = self.numVertices + 1
		newVertex = vertex.Vertex(key, pt)
		self.vertList[key] = newVertex
		self.ptList.append(pt)
		return newVertex

	def getVertex(self,n):
		if n in self.vertList:
			return self.vertList[n]
		else:
			return None

	def __contains__(self,n):
		return n in self.vertList

	def addEdge(self,f,t,cost=0):
		if f not in self.vertList:
			nv = self.addVertex(f)
		if t not in self.vertList:
			nv = self.addVertex(t)
		self.vertList[f].addNeighbor(self.vertList[t], cost)

	def getVertices(self):
		return self.vertList.keys()

	def __iter__(self):
		return iter(self.vertList.values())
	
	def dijkstra(self, start, finish):
		previous = np.zeros(len(self.vertList))
		dist = np.zeros(len(self.vertList))

		for v in self.vertList.keys():
			dist[v] = np.inf
			previous[v] = np.inf
		
		dist[start] = 0
		Q = self.vertList.keys()
		while len(Q) != 0:
			#print previous
			Q = sorted(Q, key = lambda vnum: dist[vnum] + np.linalg.norm(self.vertList[vnum].pt - self.vertList[finish].pt))
			u = Q.pop(0)
			if u == finish:
				path = []
				u = finish
				while previous[u] != np.inf:
					path.insert(0, u)
					u = previous[u]
				return path

			if  dist[u] == np.inf:
				break

			for v in self.vertList[u].getConnections():
				#print v.id
				#print self.vertList[u].getConnections()
				#print self.vertList[v.id].connectedTo
				tempdist = dist[u] + self.vertList[u].getWeight(v) + np.linalg.norm(self.vertList[v.id].pt - self.vertList[finish].pt)
				if tempdist < dist[v.id]:
					dist[v.id] = tempdist
					previous[v.id] = u
					Q = sorted(Q, key = lambda vnum: dist[vnum])


	def __repr__(self):
		string = ''
		for vert in self.vertList:
			string += repr(self.vertList[vert]) + '\n'
		return string

