# -*- coding:utf-8 -*-
#
# 寻路网格, 保存寻路数据
# 涉及到的坐标都为网格坐标
#

class Mesh(object):
	"""定义搜索网格接口类,外部继承扩展"""
	def __init__(self):
		self.weights = []

	def is_valid(self, nav_x, nav_y):
		"""路点是否有效"""
		pass

	def heuristic(self, x1, y1, x2, y2):
		"""启发函数, 计算启发值H"""
		pass

	def get_cost(self, x1, y1, x2, y2):
		"""返回路径代价"""
		pass

	def neighbors(self, nav_x, nav_y):
		"""返回周围格子坐标"""
		pass

class NavMesh(Mesh):
	''',x对应col方向，y对应row方向'''
	#寻路网格可行走编号
	MESH_WALKABLE = 0
	MESH_UNWALKABLE = 1
	def __init__(self, col, row):
		super(NavMesh, self).__init__()
		#col与row都是最大的坐标索引
		self.col = int(col)
		self.row = int(row)
		for nav_y in xrange(self.row + 1):
			self.weights.append([self.MESH_WALKABLE] * (self.col + 1))

	def is_valid(self, nav_x, nav_y):
		'''nav_x, nav_y必须都是整数'''
		if nav_x < 0 or nav_x > self.col or nav_y < 0 or nav_y > self.row:
			return False
		return self.weights[nav_y][nav_x] == self.MESH_WALKABLE

	def heuristic(self, x1, y1, x2, y2):
		"""启发函数, 计算启发值H"""
		dx = abs(x1 - x2)
		dy = abs(y1 - y2)
		#欧几里德距离
		# import math
		# dist = math.sqrt(dx * dx + dy * dy)
		#曼哈顿距离(an inadmissible heuristic)
		# dist = dx + dy
		#对角线距离
		# dist = max(dx, dy)
		# dist = min(dx, dy) * 0.4 + max(dx, dy)
		# dist = min(dx, dy) * 1.4 + (dx + dy) - 2 * min(dx, dy)
		#广搜
		dist = 0
		#解决平局
		#dist *= 1.01
		# if find_path_type == const.FIND_PATH_TYPE_ROW_PREFER:
		# 	dist = dx * 1.1 + dy
		# elif find_path_type == const.FIND_PATH_TYPE_COL_PREFER:
		# 	dist = dx + dy * 1.1
		# else:
		# 	dist = dx * 1.1 + dy
		return dist

	def get_cost(self, x1, y1, x2, y2):
		"""返回路径代价，如果有额外的代价数据，使用额外提供的代价数据，否则直接使用曼哈顿距离"""
		"""上下左右直走，代价为1.0，斜走，代价为1.4"""
		if x1 == x2 or y1 == y2:
			return 1.0
		return 1.4
		# return abs(x1 - x2) + abs(y1 - y2)

	def neighbors(self, nav_x, nav_y):
		lst = []
		#8个方向
		# xs = (-1,  0,  1, -1, 1, -1, 0, 1)
		# ys = (-1, -1, -1,  0, 0,  1, 1, 1)
		#4个方向(上下左右)
		xs = (0, -1, 1, 0)
		ys = (-1, 0, 0, 1)
		for x_s, y_s in zip(xs, ys):
			new_x, new_y = nav_x + x_s, nav_y + y_s
			if not self.is_valid(new_x, new_y):
				continue
			lst.append((new_x, new_y))
		return lst

	def set_col_row_walkable(self, nav_x, nav_y, walkable = True):
		if walkable:
			self.weights[nav_y][nav_x] = self.MESH_WALKABLE
		else:
			self.weights[nav_y][nav_x] = self.MESH_UNWALKABLE

	def destroy(self):
		self.weights = None


if __name__ == "__main__":
	tm = [
	'############################################################',
	'#..........#...............................................#',
	'#..........#...............................................#',
	'#..........#...........#...................................#',
	'#..........#...........#...................................#',
	'#..........#...........#...................................#',
	'#..........#...........#############################.......#',
	'#..........#...........#...................................#',
	'#..........#...........#...................................#',
	'#..........#...........#...................................#',
	'#..........#...........#...................................#',
	'#..........#...........#.......#############################',
	'#..........#...........#...................................#',
	'#..........#...........#...................................#',
	'#..........#...........#...................................#',
	'#..........#...........#...................................#',
	'#..........#...........#...................................#',
	'#..........#...........#...................................#',
	'#..........#...........#...................................#',
	'#..........#...........#...................................#',
	'#......................#...................................#',
	'#......................#............##############.........#',
	'#......................#............#............#.........#',
	'#......................#............#............#.........#',
	'#......................#............#............#.........#',
	'#......................#............#............#.........#',
	'#......................#............#............#.........#',
	'#......................#...................................#',
	'#......................#...................................#',
	'#......................#...................................#',
	'############################################################',
	]

	col = len(tm[0]) - 1
	row = len(tm) - 1
	mesh = NavMesh(col, row)
	for row, line in enumerate(tm):
		for col, sign in enumerate(line):
			if sign == '#':
				mesh.set_col_row_walkable(col, row, False)

	import time
	from AStar import AStar

	astar = AStar()

	sx, sy = 10, 28
	ex, ey = 40, 18
	time_begin = time.time()
	path = astar.find_path(sx, sy, ex, ey, mesh)
	time_end = time.time()

	print 'time_used', time_end - time_begin
	print "path length: %d"%(len(path))
	print "searched number: %d"%(len(astar.close))
	print "open list: %d"%(len(astar.open))

	weights = mesh.weights[:]

	for node in astar.open:
		weights[node.y][node.x] = '+'

	for node in astar.close:
		weights[node.y][node.x] = '-'

	for nav_x, nav_y in path:
		weights[nav_y][nav_x] = '*'

	for line in weights:
		for idx, sign in enumerate(line):
			if sign == mesh.MESH_WALKABLE:
				line[idx] = '.'
			elif sign == mesh.MESH_UNWALKABLE:
				line[idx] = '#'

	weights[sy][sx] = 'A'
	weights[ey][ex] = 'B'

	for line in weights:
		print ''.join(line)

	astar.destroy()
