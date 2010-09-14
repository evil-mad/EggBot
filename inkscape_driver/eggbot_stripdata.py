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
import inkex

class EggBotStripData( inkex.Effect ):
	def __init__( self ):
		inkex.Effect.__init__( self )

	def effect( self ):
		'''Main entry point: check to see which tab is selected, and act accordingly.'''
		self.svg = self.document.getroot()
		for node in self.svg.xpath( '//svg:eggbot', namespaces=inkex.NSS ):
			self.svg.remove( node )
		inkex.errormsg( gettext.gettext( "Okay, I've removed all Eggbot data from this Inkscape file.  Have a nice day!" ) )

e = EggBotStripData()
e.affect()
