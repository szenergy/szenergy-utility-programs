
#!/usr/bin/env python
#############################################################
## from http://code.activestate.com/recipes/534109-xml-to-python-data-structure/
## UTM


import re
import xml.sax.handler
import matplotlib.pyplot as plt
import utm

def xml2obj(src):
    """
    A simple function to converts XML data into native Python object.
    """

    non_id_char = re.compile('[^_0-9a-zA-Z]')
    def _name_mangle(name):
        return non_id_char.sub('_', name)

    class DataNode(object):
        def __init__(self):
            self._attrs = {}    # XML attributes and child elements
            self.data = None    # child text data
        def __len__(self):
            # treat single element as a list of 1
            return 1
        def __getitem__(self, key):
            if isinstance(key, str):  # basestring python 2 version
                return self._attrs.get(key,None)
            else:
                return [self][key]
        def __contains__(self, name):
            return self._attrs.has_key(name)
        def __nonzero__(self):
            return bool(self._attrs or self.data)
        def __getattr__(self, name):
            if name.startswith('__'):
                # need to do this for Python special methods???
                raise AttributeError(name)
            return self._attrs.get(name,None)
        def _add_xml_attr(self, name, value):
            if name in self._attrs:
                # multiple attribute of the same name are represented by a list
                children = self._attrs[name]
                if not isinstance(children, list):
                    children = [children]
                    self._attrs[name] = children
                children.append(value)
            else:
                self._attrs[name] = value
        def __str__(self):
            return self.data or ''
        def __repr__(self):
            items = sorted(self._attrs.items())
            if self.data:
                items.append(('data', self.data))
            return u'{%s}' % ', '.join([u'%s:%s' % (k,repr(v)) for k,v in items])

    class TreeBuilder(xml.sax.handler.ContentHandler):
        def __init__(self):
            self.stack = []
            self.root = DataNode()
            self.current = self.root
            self.text_parts = []
        def startElement(self, name, attrs):
            self.stack.append((self.current, self.text_parts))
            self.current = DataNode()
            self.text_parts = []
            # xml attributes --> python attributes
            for k, v in attrs.items():
                self.current._add_xml_attr(_name_mangle(k), v)
        def endElement(self, name):
            text = ''.join(self.text_parts).strip()
            if text:
                self.current.data = text
            if self.current._attrs:
                obj = self.current
            else:
                # a text only node is simply represented by the string
                obj = text or ''
            self.current, self.text_parts = self.stack.pop()
            self.current._add_xml_attr(_name_mangle(name), obj)
        def characters(self, content):
            self.text_parts.append(content)

    builder = TreeBuilder()
    if isinstance(src,str): # basestring python 2 version
        xml.sax.parseString(src, builder)
    else:
        xml.sax.parse(src, builder)
    values_view = builder.root._attrs.values()
    value_iterator = iter(values_view)
    first_value = next(value_iterator)
    # return builder.root._attrs.values()[0] # python 2 version
    return first_value



#############################################################

#src = file("map_campus_gyor01.osm")  # basestring python 2 version
#src = open("../osm_files/map_campus_zala_egyetemi01.osm")
src = open("../osm_files/map_campus_gyor01.osm")
src = open("../osm_files/map_campus_zala_smart_city01.osm")
myMap = xml2obj(src)
#main()


#def main():
# make dictionary of node IDs
nodes = {}
for node in myMap['node']:
    nodes[node['id']] = node
    
ways = {}
for way in myMap['way']:
    ways[way['id']]=way

    
print(len(nodes))
print(len(ways))

# try to find a list of all intersections

# build matrix of first and last node ids for each way

print('Enumerating end nodes.')
endNodes = {}
wayType  = {}
for wayID in ways.keys():
    endNodes[wayID] = [ ways[wayID]['nd'][0], ways[wayID]['nd'][-1]]    
    wayTags         = ways[wayID]['tag']
    if not wayTags==None:
        hwyTypeList  = [d['v'] for d in wayTags if d['k']=='highway']
        if len(hwyTypeList)>0:
                wayType[wayID] = hwyTypeList[0]

print('Done enumerating end nodes')
print(wayType)
print(set([wayType[k] for k in wayType.keys()]))
roadTypes = set([wayType[k] for k in wayType.keys()])


for idx,nodeID in enumerate(nodes.keys()):
    if idx==0:
        minX = float(nodes[nodeID]['lon'])
        maxX = float(nodes[nodeID]['lon'])
        minY = float(nodes[nodeID]['lat'])
        maxY = float(nodes[nodeID]['lat'])
    else:
        minX = min(minX,float(nodes[nodeID]['lon']))
        maxX = max(maxX,float(nodes[nodeID]['lon']))
        minY = min(minY,float(nodes[nodeID]['lat']))
        maxY = max(maxY,float(nodes[nodeID]['lat']))

minLon = float(myMap['bounds']['minlon'])
maxLon = float(myMap['bounds']['maxlon'])
minLat = float(myMap['bounds']['minlat'])
maxLat = float(myMap['bounds']['maxlat'])

minX, minY, _, _ = utm.from_latlon(minLat, minLon)
maxX, maxY, _, _ = utm.from_latlon(maxLat, maxLon)

fig = plt.figure()
ax = fig.add_subplot()
try:
    ax.axis("equal")
    ax.axis([minX,maxX,minY,maxY])
except:
    print("python2")
plt.grid()
for idx,nodeID in enumerate(wayType.keys()):
    try:
        oldX = None
        oldY = None
        for nCnt,nID in enumerate(ways[nodeID]['nd']):
            llat = float(nodes[nID['ref']]['lat'])
            #y = float(nodes[nID['ref']]['lat'])
            llon = float(nodes[nID['ref']]['lon'])
            #x = float(nodes[nID['ref']]['lon'])
            x, y, _, _ = utm.from_latlon(llat, llon)
            #print(x, y)
            if nCnt<(len(ways[nodeID]['nd'])-1):
                # all except the last segment in road way
                plt.plot([oldX,x],[oldY,y],
                    marker          = '',
                    linestyle       = '-',
                    linewidth       = 4,
                    color           = (0.1, 0.5, 0.8),
                    )
            else:
                # last segment in road way
                plt.plot([oldX,x],[oldY,y],
                    marker          = '',
                    linestyle       = '-',
                    linewidth       = 4,
                    color           = (0.1, 0.5, 0.8),
                    )                    
            oldX = x
            oldY = y
    except KeyError:
        pass
    if idx%100 == 0:
        print(idx)
        
print('Done Plotting')
plt.show()



def nodeDist(nodes,oldNodeID,newNodeID):
    y0 = float(nodes[oldNodeID]['lat'])
    x0 = float(nodes[oldNodeID]['lon'])
    y1 = float(nodes[newNodeID]['lat'])
    x1 = float(nodes[newNodeID]['lon'])

    return ((x0-x1)**2+(y0-y1)**2)**(0.5)


