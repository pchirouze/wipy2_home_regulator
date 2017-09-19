''' Docstring here '''
#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  sans titre.py
# 
#  Copyright 2017 Patrice <patrice.chirouze@free.fr>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#
#
import sys

def main(args):
    ''' Docstring here '''
    print ('Hello world')
    var_x=0
    for  k in range(0,10,1):
        var_x += 1


    return 0 

if __name__ == '__main__':
    sys.exit(main(sys.argv))
