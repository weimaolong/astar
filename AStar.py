# -*- coding: utf-8 -*-
#
# A*寻路算法实现
#

class Node(object):
	"""搜索节点"""
	def __init__(self, parent, x, y):
		self.parent = parent
		self.x = x
		self.y = y
		self.g = 0
		self.h = 0
		self.f = 0

	def destroy(self):
		self.parent = None

	def __str__(self):
		return 'node{x:' + str(self.x) + ' y:' + str(self.y) + ' g:' + str(self.g) + ' h:' + str(self.h) + ' f:' + str(self.f) + '}'

class AStar(object):
	"""A*算法实现"""
	def __init__(self):
		self.open = []
		self.close = []
		self.path = []
		self.clear()

	def clear(self):
		for node in self.open:
			node.destroy()
		for node in self.close:
			node.destroy()
		self.open = []
		self.close = []
		self.path = []
		self.mesh = None

	def find_path(self, sx, sy, ex, ey, mesh):
		"""查找路径的入口函数"""
		self.sx = int(sx)
		self.sy = int(sy)
		self.ex = int(ex)
		self.ey = int(ey)
		self.mesh = mesh
		node = self._create_node(None, self.sx, self.sy)
		self._add_to_open(node)
		while self.open:
			idx, node = self._get_best()
			if self._is_goal(node):
				self._reconstruct_path(node)
				break
			self._remove_from_open(idx, node)
			self._add_to_close(node)
			self._extend_round(node)
		path = self.path[:]
		return path

	def _add_to_open(self, node):
		if node not in self.open:
			self.open.append(node)

	def _remove_from_open(self, node_idx, node):
		del self.open[node_idx]

	def _add_to_close(self, node):
		if node not in self.close:
			self.close.append(node)

	def _create_node(self, parent, x, y):
		node = Node(parent, x, y)
		if parent is None:
			node.g = 0
		else:
			node.g = parent.g + self.mesh.get_cost(parent.x, parent.y, x, y)
		node.h = self.mesh.heuristic(x, y, self.ex, self.ey)
		node.f = node.g + node.h
		return node

	def _is_goal(self, node):
		if node.x == self.ex and node.y == self.ey:
			return True
		return False

	def _is_in_close(self, x, y):
		for node in self.close:
			if x == node.x and y == node.y:
				return True
		return False

	def _is_in_open(self, x, y):
		for node in self.open:
			if x == node.x and y == node.y:
				return node
		return None

	def _extend_round(self, cur_node):
		lst = self.mesh.neighbors(cur_node.x, cur_node.y)
		for x, y in lst:
			if self._is_in_close(x, y):
				continue
			node = self._is_in_open(x, y)
			if node != None:
				#如果在open_list中，计算新的g值
				new_g = cur_node.g + self.mesh.get_cost(cur_node.x, cur_node.y, x, y)
				if node.g > new_g:
					#经过node针对该x,y节点有更好的路径，更新G值
					self._update_node(node, cur_node, new_g)
					continue
			else:
				new_node = self._create_node(cur_node, x, y)
				self._add_to_open(new_node)

	def _update_node(self, node, parent, new_g):
		'''更新openlist中的节点'''
		node.g = new_g
		node.parent = parent
		node.f = node.g + node.h

	def _reconstruct_path(self, node):
		"""从结束点回溯到起始点"""
		while node:
			self.path.append((node.x, node.y))
			node = node.parent
		self.path.reverse()

	def _get_best(self):
		best_node_idx = -1
		best_node = None
		best_f = -1 & 0xffffffff
		for idx, node in enumerate(self.open):
			if node.f < best_f:
				best_node_idx = idx
				best_node = node
				best_f = node.f
		return best_node_idx, best_node

	def destroy(self):
		self.clear()
