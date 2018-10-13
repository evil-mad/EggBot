# coding=utf-8
# EggBot Path Ordering extension
# This extension tries to re-order the document's paths to improve
# the plotting time by plotting nearby paths consecutively.
#
# Written by Matthew Beckler for the EggBot project.
# Email questions and comments to matthew at mbeckler dot org
# Modified by Romain Testuz
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

import gettext
import math
import random
import sys

import inkex
import simplepath
import simpletransform

def dist(x0, y0, x1, y1):
	return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

def dist_t(x, y):
	return dist(x[0], x[1], y[0], y[1])

class Path:
	def __init__(self, id, p1, p2, reversed=False):
		self.id = id
		self.p1 = p1
		self.p2 = p2
		self.reversed = reversed

	def __eq__(self, other): #ids must be unique
		return self.id == other.id

	def __str__(self):
		return "{}: {} {}".format(self.id, self.get_start(), self.get_end())

	def get_start(self):
		return self.p1 if not self.reversed else self.p2

	def get_end(self):
		return self.p2 if not self.reversed else self.p1

	def reverse(self):
		self.reversed = not self.reversed

	def dist_to_start(self, otherPath):
		return dist_t(self.get_end(), otherPath.get_start())

	def dist_to_end(self, otherPath):
		return dist_t(self.get_end(), otherPath.get_end())

def find_ordering(objlist, allowReverse):
	"""
	Takes a list of Paths, and finds the best ordering.
	Uses a greedy algorithm which can reverse the path direction if necessary
	Returns a list of (id, reverse), as well as the original and optimized
	"air distance" which is just the distance traveled in the air. Reverse indicate if the path must be reversed
	"""

	start = (0.0, 0.0) #Start point TODO 0,0 is not always top left of the page

	# let's figure out the default in-air length (this is not meaningful as we are using the dictionary ordering)
	air_length_default = 0
	old = start

	for i, path in enumerate(objlist[1:]):
		air_length_default += objlist[i-1].dist_to_start(path)

	air_length_ordered = 0

	sort_list = []
	prev = Path("", start, start)

	# for the previous end point, iterate over each remaining path and pick the closest starting point or ending point if allowed
	while objlist:
		min_distance = sys.float_info.max # The biggest number possible
		for path in objlist:
			dist_to_start = prev.dist_to_start(path)
			dist_to_end = prev.dist_to_end(path)

			if dist_to_start < min_distance:
				min_distance = dist_to_start
				min_path = path

			if allowReverse and dist_to_end < min_distance:
				min_distance = dist_to_end
				path.reverse()
				min_path = path

		air_length_ordered += min_distance
		sort_list.append(min_path)
		objlist.remove(min_path)
		prev = min_path

	return sort_list, air_length_default, air_length_ordered

def conv(x, y, trans_matrix=None):
	"""
	apply a translation matrix to an (x, y) pair
	I'm sure there is a better way to do this using simpletransform or it's ilk
	"""

	if trans_matrix:
		xt = trans_matrix[0][0] * x + trans_matrix[0][1] * y + trans_matrix[0][2]
		yt = trans_matrix[1][0] * x + trans_matrix[1][1] * y + trans_matrix[1][2]
		return xt, yt
	else:
		return x, y

def reversePath(path):
	#Input: path in simplepath format
	#Returns the reversed path in a svg string format
	#In case of error the path is returned unchanged
	#Some commands like A, H, V are not supported
	#Adapted from https://github.com/Pomax/svg-path-reverse/blob/gh-pages/reverse.js

	#Unpack sublists into a single list
	flattenedPath = [item for sublist in path for subsublist in sublist for item in subsublist]
	reversedPath = []

	i = 0
	while i < len(flattenedPath):
		term = flattenedPath[i]
		#At this point the next term must be a letter because the coordinates must have all been read
		if term == "C":
			pairs = 3; shift = 2
		elif term == "Q":
			pairs = 2; shift = 1
		elif term == "L":
			pairs = 1; shift = 1
		elif term == "M":
			pairs = 1; shift = 0
		elif term == "Z":
			reversedPath[0] = "Z"; i += 1; continue
		else:
			inkex.errormsg("Cannot reverse path, unknown command: {} or malformed path: {}".format(term, flattenedPath))
			return ' '.join(str(e) for e in flattenedPath)#to string

		if pairs == shift:
			reversedPath.append(term)

		for pair in range(0, pairs):
			if pair == shift:
				reversedPath.append(term)
			i += 1
			x = flattenedPath[i]
			i += 1
			y = flattenedPath[i]
			reversedPath.append(y)
			reversedPath.append(x)
		i += 1

	reversedPath.append("M")
	reversedPath = list(reversed(reversedPath))
	if reversedPath[-1] == "M":#Only remove the last element if it's not a Z
		reversedPath = reversedPath[:-1]

	return ' '.join(str(e) for e in reversedPath) # to string

class EggBotReorderPaths(inkex.Effect):
	def __init__(self):
		inkex.Effect.__init__(self)
		self.OptionParser.add_option("-r", "--allowReverse", action="store", type="inkbool",
                        dest="allowReverse", default=True, help="Allow path reversal")

	def get_start_end(self, node):
		"""Given a node, return the start and end points"""
		if node.tag != inkex.addNS('path', 'svg'):
			inkex.errormsg("Groups are not supported, please ungroup for better results")
			return (0, 0, 0, 0)

		d = node.get('d')
		sp = simplepath.parsePath(d)

		# simplepath converts coordinates to absolute and cleans them up, but
		# these are still some big assumptions here, are they always valid? TODO
		startX = sp[0][1][0]
		startY = sp[0][1][1]
		if sp[-1][0] == 'Z':
			# go back to start
			endX = startX
			endY = startY
		else:
			endX = sp[-1][1][-2]
			endY = sp[-1][1][-1]

		transform = node.get('transform')
		if transform:
			transform = simpletransform.parseTransform(transform)

		sx, sy = conv(startX, startY, transform)
		ex, ey = conv(endX, endY, transform)
		return (sx, sy, ex, ey)

	def effect(self):
		"""This is the main entry point"""

		# based partially on the restack.py extension
		if len(self.selected) > 0:
			# TODO check for non-path elements?
			# TODO it seems like the order of selection is not consistent
			# => self.selected is a dict so it has no meaningful order and should not be used to evaluate the original path length

			#fid = open("/home/matthew/debug.txt", "w")

			# for each selected item
			# I can think of two options:
			# 1. Iterate over all paths in root, then iterate over all layers, and their paths
			# 2. Some magic with xpath? (would this limit us to specific node types?)

			objlist = []
			for id, node in self.selected.iteritems():
				(sx, sy, ex, ey) = self.get_start_end(node)
				path = Path(id, (sx, sy), (ex, ey))
				objlist.append(path)

			# sort / order the objects
			sort_order, air_distance_default, air_distance_ordered = find_ordering(objlist, self.options.allowReverse)

			reverseCount = 0
			for path in sort_order:
				node = self.selected[path.id]
				if node.tag == inkex.addNS('path', 'svg'):
					node_sp = simplepath.parsePath(node.get('d'))
					if(path.reversed):
						node_sp_string = reversePath(node_sp)
						reverseCount += 1
					else:
						node_sp_string = simplepath.formatPath(node_sp)

					node.set('d', node_sp_string)

				#keep in mind the different selected ids might have different parents
				self.getParentNode(node).append(node)

			inkex.errormsg("Reversed {} paths.".format(reverseCount))
			#fid.close()

			if air_distance_default > 0:  # don't divide by zero. :P
				improvement_pct = 100 * (( air_distance_default - air_distance_ordered) / (air_distance_default))
				inkex.errormsg(gettext.gettext("Selected paths have been reordered and optimized for quicker EggBot plotting.\n\nDefault air-distance: %d\nOptimized air-distance: %d\nDistance reduced by: %1.2d%%\n\nHave a nice day!" % (air_distance_default, air_distance_ordered, improvement_pct)))
			else:
				inkex.errormsg(gettext.gettext("Unable to start. Please select multiple distinct paths. :)"))


e = EggBotReorderPaths()
e.affect()
